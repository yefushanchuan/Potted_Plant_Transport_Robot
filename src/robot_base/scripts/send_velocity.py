#!/usr/bin/env python3

import serial
import struct
import argparse
import sys
import time
import math

FRAME_HEAD_1 = 0xA0
FRAME_HEAD_2 = 0x0A
PROTOCOL_VER = 0x01
UART_TYPE_COMMAND = 0x81
UART_TYPE_TELEMETRY = 0x01
UART_TYPE_ACK = 0x02

TAG_V_LINEAR_MM_S = 0x01
TAG_W_ANGULAR_MRAD_S = 0x02
TAG_Z_LIFT_MM = 0x03
TAG_HEALTH_WORD = 0x14
TAG_ALARM_INFO = 0x15
TAG_BATT_SOC_X100 = 0x21
TAG_BUCKET_VOLUME_ML = 0x23

FRAME_FIXED_HEADER_LEN = 7
FRAME_MIN_SIZE = FRAME_FIXED_HEADER_LEN + 2


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def append_int32_tlv(payload, tag, value):
    payload.append(tag)
    payload.append(4)
    payload.extend(struct.pack('<i', value))


def build_command_frame(v_linear_mm_s, w_angular_mrad_s, seq_id=0):
    payload = []
    append_int32_tlv(payload, TAG_V_LINEAR_MM_S, int(v_linear_mm_s))
    append_int32_tlv(payload, TAG_W_ANGULAR_MRAD_S, int(w_angular_mrad_s))

    frame = bytearray()
    frame.append(FRAME_HEAD_1)
    frame.append(FRAME_HEAD_2)
    frame.append(PROTOCOL_VER)
    frame.append(seq_id & 0xFF)
    frame.append(UART_TYPE_COMMAND)
    frame.append(len(payload) & 0xFF)
    frame.append((len(payload) >> 8) & 0xFF)
    frame.extend(payload)

    crc = crc16_modbus(frame[2:])
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)

    return bytes(frame)


def bytes_to_hex_string(data):
    return ' '.join(f'{b:02X}' for b in data)


def read_tlv_data(payload, payload_len):
    result = {}
    offset = 0
    while offset + 2 <= payload_len:
        tag = payload[offset]
        length = payload[offset + 1]
        if offset + 2 + length > payload_len:
            break
        data = payload[offset + 2:offset + 2 + length]

        if tag == TAG_V_LINEAR_MM_S and length == 4:
            result['v_linear_mm_s'] = struct.unpack('<i', bytes(data))[0]
        elif tag == TAG_W_ANGULAR_MRAD_S and length == 4:
            result['w_angular_mrad_s'] = struct.unpack('<i', bytes(data))[0]
        elif tag == TAG_Z_LIFT_MM and length == 4:
            result['z_lift_mm'] = struct.unpack('<i', bytes(data))[0]
        elif tag == TAG_HEALTH_WORD and length == 2:
            result['health_word'] = struct.unpack('<H', bytes(data))[0]
        elif tag == TAG_ALARM_INFO and length == 2:
            result['alarm_info'] = struct.unpack('<H', bytes(data))[0]
        elif tag == TAG_BATT_SOC_X100 and length == 2:
            result['batt_soc_x100'] = struct.unpack('<H', bytes(data))[0]
        elif tag == TAG_BUCKET_VOLUME_ML and length == 2:
            result['bucket_volume_ml'] = struct.unpack('<H', bytes(data))[0]

        offset += 2 + length

    return result


def parse_frame(data):
    if len(data) < FRAME_MIN_SIZE:
        return None

    if data[0] != FRAME_HEAD_1 or data[1] != FRAME_HEAD_2:
        return None

    if len(data) < FRAME_FIXED_HEADER_LEN:
        return None

    version = data[2]
    seq_id = data[3]
    frame_type = data[4]
    payload_len = data[5] | (data[6] << 8)

    frame_len = FRAME_FIXED_HEADER_LEN + payload_len + 2
    if len(data) < frame_len:
        return None

    if version != PROTOCOL_VER:
        return None

    crc_in_frame = data[FRAME_FIXED_HEADER_LEN + payload_len] | \
                   (data[FRAME_FIXED_HEADER_LEN + payload_len + 1] << 8)
    crc_calc = crc16_modbus(data[2:FRAME_FIXED_HEADER_LEN + payload_len])

    if crc_calc != crc_in_frame:
        return None

    payload = data[FRAME_FIXED_HEADER_LEN:FRAME_FIXED_HEADER_LEN + payload_len]

    result = {
        'version': version,
        'seq_id': seq_id,
        'type': frame_type,
        'payload_len': payload_len,
        'raw': bytes_to_hex_string(data)
    }

    if frame_type == UART_TYPE_TELEMETRY:
        tlv_data = read_tlv_data(payload, payload_len)
        result['telemetry'] = tlv_data
    elif frame_type == UART_TYPE_ACK:
        if len(payload) >= 3 and payload[0] == 0x30 and payload[1] == 1:
            result['ack_result'] = payload[2]

    return result


class PositionTracker:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        self.last_vx = 0.0
        self.last_vth = 0.0

    def update(self, vx_mm_s, vth_mrad_s, dt=None):
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

        vx = vx_mm_s / 1000.0
        vth = vth_mrad_s / 1000.0

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += vth * dt

        self.last_vx = vx_mm_s
        self.last_vth = vth_mrad_s

        return self.x, self.y, self.yaw

    def get_position(self):
        return self.x, self.y, self.yaw


def send_velocity_and_track(serial_port, baudrate, v_linear_mm_s, w_angular_mrad_s,
                           duration=2.0, update_rate=20):
    try:
        ser = serial.Serial(port=serial_port, baudrate=baudrate, timeout=0.1)
        time.sleep(0.5)

        tracker = PositionTracker()
        seq_id = 0
        start_time = time.time()

        print(f"{'Time(s)':>8} | {'VX(mm/s)':>10} | {'W(mrad/s)':>12} | {'X(m)':>10} | {'Y(m)':>10} | {'Yaw(rad)':>10} | {'RX Frame':>20}")
        print("-" * 100)

        last_print_time = start_time
        print_interval = 1.0 / update_rate

        while time.time() - start_time < duration:
            frame = build_command_frame(v_linear_mm_s, w_angular_mrad_s, seq_id)
            seq_id = (seq_id + 1) & 0xFF

            ser.write(frame)

            rx_buffer = bytearray()
            while ser.in_waiting > 0:
                rx_buffer.extend(ser.read(ser.in_waiting))

            current_time = time.time()

            if rx_buffer:
                while len(rx_buffer) >= FRAME_MIN_SIZE:
                    frame_data = parse_frame(rx_buffer)
                    if frame_data is None:
                        rx_buffer.pop(0)
                        continue

                    if 'telemetry' in frame_data:
                        telemetry = frame_data['telemetry']
                        rx_vx = telemetry.get('v_linear_mm_s', 0)
                        rx_vth = telemetry.get('w_angular_mrad_s', 0)

                        dt = current_time - last_print_time
                        if dt >= print_interval or True:
                            x, y, yaw = tracker.update(rx_vx, rx_vth, dt)
                            rx_hex = frame_data['raw'][:20] + "..." if len(frame_data['raw']) > 20 else frame_data['raw']
                            elapsed = current_time - start_time
                            print(f"{elapsed:>8.2f} | {rx_vx:>10.2f} | {rx_vth:>12.2f} | "
                                  f"{x/1000.0:>10.4f} | {y/1000.0:>10.4f} | {yaw:>10.4f} | {rx_hex}")

                            last_print_time = current_time

                    rx_buffer = rx_buffer[len(rx_buffer):]
                    break
            else:
                dt = current_time - last_print_time
                if dt >= print_interval:
                    x, y, yaw = tracker.update(v_linear_mm_s, w_angular_mrad_s, dt)
                    elapsed = current_time - start_time
                    print(f"{elapsed:>8.2f} | {v_linear_mm_s:>10.2f} | {w_angular_mrad_s:>12.2f} | "
                          f"{x/1000.0:>10.4f} | {y/1000.0:>10.4f} | {yaw:>10.4f} | {'(sent only)':>20}")
                    last_print_time = current_time

            time.sleep(0.05)

        final_x, final_y, final_yaw = tracker.get_position()
        print("-" * 100)
        print(f"Final Position: X={final_x/1000.0:.4f}m, Y={final_y/1000.0:.4f}m, Yaw={final_yaw:.4f}rad")

        ser.close()
        return True

    except Exception as e:
        print(f"Error: {e}")
        return False


def send_velocity(serial_port, baudrate, v_linear_mm_s, w_angular_mrad_s):
    try:
        ser = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
        frame = build_command_frame(v_linear_mm_s, w_angular_mrad_s)
        print(f"Sending frame: {bytes_to_hex_string(frame)}")
        ser.write(frame)
        print("Frame sent successfully")

        rx_buffer = bytearray()
        start_time = time.time()
        while time.time() - start_time < 1.0:
            if ser.in_waiting > 0:
                rx_buffer.extend(ser.read(ser.in_waiting))
                while len(rx_buffer) >= FRAME_MIN_SIZE:
                    frame_data = parse_frame(rx_buffer)
                    if frame_data is None:
                        rx_buffer.pop(0)
                        continue
                    print(f"Received: {frame_data['raw']}")
                    if 'telemetry' in frame_data:
                        print(f"Telemetry: {frame_data['telemetry']}")
                    rx_buffer = rx_buffer[len(rx_buffer):]
                    break
            time.sleep(0.01)

        ser.close()
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Send velocity command to STM32 and track position')
    parser.add_argument('--port', type=str, default='/dev/STM32',
                        help='Serial port name (default: /dev/STM32)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--v', type=float, default=100.0,
                        help='Linear velocity in mm/s')
    parser.add_argument('--w', type=float, default=0.0,
                        help='Angular velocity in mrad/s')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Duration to run and track position (default: 2.0 seconds)')
    parser.add_argument('--track', action='store_true',
                        help='Enable position tracking mode')

    args = parser.parse_args()

    if args.track:
        success = send_velocity_and_track(args.port, args.baudrate, args.v, args.w, args.duration)
    else:
        time_start = time.time()
        now = time.time()
        while now - time_start < 2:
            success = send_velocity(args.port, args.baudrate, args.v, args.w)
            now = time.time()
        success = send_velocity(args.port, args.baudrate, 0.0, 0.0)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
