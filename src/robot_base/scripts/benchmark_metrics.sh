#!/bin/bash
# 实验指标记录脚本
# 用法: bash benchmark_metrics.sh [输出目录] [采样间隔秒]
# 记录 fastlio2 和 EKF 的 CPU、odom 频率、odom 延迟

OUT_DIR="${1:-$(pwd)/benchmark_result}"
INTERVAL="${2:-5}"

mkdir -p "$OUT_DIR"

CPU_LOG="$OUT_DIR/cpu.csv"
HZ_LOG="$OUT_DIR/frequency.csv"
DELAY_LOG="$OUT_DIR/delay.csv"

echo "timestamp,fastlio2_cpu%,ekf_cpu%" > "$CPU_LOG"
echo "timestamp,node,avg_rate_hz,window_size" > "$HZ_LOG"
echo "timestamp,node,avg_delay_ms,std_delay_ms,window_size" > "$DELAY_LOG"

echo "指标记录启动，输出目录: $OUT_DIR"
echo "采样间隔: ${INTERVAL}s"
echo "Ctrl+C 停止记录"
echo "---"

RUNNING=true

cleanup() {
    RUNNING=false
    # 用 pkill 杀掉所有 ros2 topic hz/delay 子进程
    pkill -P $$ 2>/dev/null
    wait 2>/dev/null
    echo "---"
    echo "记录结束，文件保存在: $OUT_DIR"
    echo "  $CPU_LOG"
    echo "  $HZ_LOG"
    echo "  $DELAY_LOG"
}
trap cleanup EXIT INT TERM

# 后台运行 ros2 topic hz
ros2 topic hz /fastlio2/lio_odom 2>&1 | while read -r line; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    rate=$(echo "$line" | grep -oP 'average rate: \K[0-9.]+')
    window=$(echo "$line" | grep -oP 'window: \K[0-9]+')
    if [ -n "$rate" ]; then
        echo "$ts,fastlio2,$rate,$window" >> "$HZ_LOG"
    fi
done &
PID_HZ_FASTLIO=$!

ros2 topic hz /odom 2>&1 | while read -r line; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    rate=$(echo "$line" | grep -oP 'average rate: \K[0-9.]+')
    window=$(echo "$line" | grep -oP 'window: \K[0-9]+')
    if [ -n "$rate" ]; then
        echo "$ts,ekf,$rate,$window" >> "$HZ_LOG"
    fi
done &
PID_HZ_EKF=$!

# 后台运行 ros2 topic delay
ros2 topic delay /fastlio2/lio_odom 2>&1 | while read -r line; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    delay=$(echo "$line" | grep -oP 'average delay: \K[0-9.]+')
    std=$(echo "$line" | grep -oP '\tstd: \K[0-9.]+')
    window=$(echo "$line" | grep -oP 'window: \K[0-9]+')
    if [ -n "$delay" ]; then
        echo "$ts,fastlio2,$delay,$std,$window" >> "$DELAY_LOG"
    fi
done &
PID_DELAY_FASTLIO=$!

ros2 topic delay /odom 2>&1 | while read -r line; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    delay=$(echo "$line" | grep -oP 'average delay: \K[0-9.]+')
    std=$(echo "$line" | grep -oP '\tstd: \K[0-9.]+')
    window=$(echo "$line" | grep -oP 'window: \K[0-9]+')
    if [ -n "$delay" ]; then
        echo "$ts,ekf,$delay,$std,$window" >> "$DELAY_LOG"
    fi
done &
PID_DELAY_EKF=$!

# 主循环：周期采样 CPU
while $RUNNING; do
    ts=$(date '+%Y-%m-%d %H:%M:%S')
    fastlio2_cpu=$(ps -C fastlio2_node -o %cpu= 2>/dev/null | tr -d ' ')
    ekf_cpu=$(ps -C ekf_node -o %cpu= 2>/dev/null | tr -d ' ')
    fastlio2_cpu="${fastlio2_cpu:-0}"
    ekf_cpu="${ekf_cpu:-0}"
    echo "$ts,$fastlio2_cpu,$ekf_cpu" >> "$CPU_LOG"
    sleep "$INTERVAL" &
    wait $!
done
