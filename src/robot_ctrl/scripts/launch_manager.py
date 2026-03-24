#!/usr/bin/env python3
"""Launch 管理器示例。

该脚本演示如何在单一进程中统一启动/关闭/重启多份 launch 文件，
并通过 ROS2 服务暴露控制接口，行为树动作节点可以直接调用这些服务
来执行批量操作。

默认会管理以下 launch（路径可按需调整）：
  - robot_bringup:1_agrobot_base_bringup.launch.py
  - robot_bringup:agrobot_localization_bringup.launch.py
  - robot_bringup:3_agrobot_nav_by_route_bringup.launch.py
  - robot_bringup:pheno_module_controller.launch.py
  - robot_bringup:2_agrobot_mapping_bringup.launch.py

服务接口：
  - /launch_manager/start              (std_srvs/Trigger)
  - /launch_manager/stop               (std_srvs/Trigger)
  - /launch_manager/restart            (std_srvs/Trigger)
  - /launch_manager/start_navigation   (std_srvs/Trigger)
  - /launch_manager/start_mapping      (std_srvs/Trigger)

参数：
  - default_mode (string, default: navigation)
  - default_launch_package (string, default: robot_bringup)
  - common_launches (string, 逗号分隔，支持 package:launch_file)
  - navigation_launches (string, 逗号分隔，支持 package:launch_file)
  - mapping_launches (string, 逗号分隔，支持 package:launch_file)

行为树中可创建动作节点调用上述服务，当检测到初始化失败时调用
restart 即可完成所有 launch 的重新加载。需要切换模式时调用
start_navigation 或 start_mapping。
"""

import os
import shlex
import signal
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import (  # type: ignore
    PackageNotFoundError,
    get_package_share_directory,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class LaunchSpec:
    """记录单个 launch 的元信息。"""

    name: str
    package: str
    launch_file: str
    arguments: Dict[str, str]


class LaunchManagerNode(Node):
    """统一管理多份 launch 的 ROS2 节点。"""

    DEFAULT_LAUNCH_PACKAGE = "robot_bringup"
    DEFAULT_COMMON_LAUNCHES = "1_agrobot_base_bringup.launch.py"
    DEFAULT_NAVIGATION_LAUNCHES = ",".join([
        "agrobot_localization_bringup.launch.py",
        "3_agrobot_nav_by_route_bringup.launch.py",
        "pheno_module_controller.launch.py",
    ])
    DEFAULT_MAPPING_LAUNCHES = "2_agrobot_mapping_bringup.launch.py"

    def __init__(self) -> None:
        super().__init__("launch_manager")

        self.common_specs, self.mode_specs = self._load_default_specs()
        self.active_mode = self._init_mode()
        self.processes: Dict[str, subprocess.Popen] = {}
        self.pid_files: Dict[str, Path] = {}
        self.pid_dir = Path(tempfile.gettempdir()) / "launch_manager_pids"
        self.pid_dir.mkdir(parents=True, exist_ok=True)
        self.terminal = self._select_terminal()
        if self.terminal:
            self.get_logger().info(
                f"将通过终端 {self.terminal} 启动各 launch，方便查看日志。"
            )
        self.running = False

        self.start_srv = self.create_service(Trigger, "launch_manager/start", self.handle_start)
        self.stop_srv = self.create_service(Trigger, "launch_manager/stop", self.handle_stop)
        self.restart_srv = self.create_service(Trigger, "launch_manager/restart", self.handle_restart)
        self.start_nav_srv = self.create_service(
            Trigger, "launch_manager/start_navigation", self.handle_start_navigation)
        self.start_mapping_srv = self.create_service(
            Trigger, "launch_manager/start_mapping", self.handle_start_mapping)

        self.state_pub = self.create_publisher(String, "launch_manager/state", 10)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"LaunchManagerNode ready. Default mode: {self.active_mode}. "
            "Call /launch_manager/start to begin.")

    def _init_mode(self) -> str:
        """初始化启动模式。"""

        self.declare_parameter("default_mode", "navigation")
        raw_mode = self.get_parameter("default_mode").get_parameter_value().string_value
        mode = self._normalize_mode(raw_mode)
        if mode not in self.mode_specs:
            self.get_logger().warn(
                f"Unknown default_mode '{raw_mode}', fallback to 'navigation'.")
            mode = "navigation"
        return mode

    def _select_terminal(self) -> str:
        """根据环境变量与可用性选择终端程序。"""

        preferred = os.environ.get("LAUNCH_MANAGER_TERMINAL", "").strip()
        if preferred.lower() in {"none", "off", "false", "0"}:
            self.get_logger().info(
                "LAUNCH_MANAGER_TERMINAL disabled by environment, using background mode."
            )
            return ""

        # systemd 等无图形会话环境下不要尝试拉起终端，避免 "Cannot open display"
        has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if not has_display:
            if preferred:
                self.get_logger().warn(
                    "No DISPLAY/WAYLAND_DISPLAY in environment, ignore terminal request "
                    f"'{preferred}' and use background mode."
                )
            return ""

        candidates = [preferred] if preferred else []
        candidates.extend(["gnome-terminal", "konsole", "xterm"])

        for candidate in candidates:
            if not candidate:
                continue
            cmd = candidate.split()[0]
            if shutil.which(cmd):
                return candidate

        if preferred:
            self.get_logger().warn(
                f"终端程序 {preferred} 不存在，将使用后台进程模式。"
            )
        return ""

    # ==================================================================
    # Launch 构建与运行逻辑
    # ==================================================================
    def _load_default_specs(self) -> tuple[List[LaunchSpec], Dict[str, List[LaunchSpec]]]:
        """从参数加载 launch 列表，未配置时使用默认值。"""

        self.declare_parameter("default_launch_package", self.DEFAULT_LAUNCH_PACKAGE)
        self.declare_parameter("common_launches", self.DEFAULT_COMMON_LAUNCHES)
        self.declare_parameter("navigation_launches", self.DEFAULT_NAVIGATION_LAUNCHES)
        self.declare_parameter("mapping_launches", self.DEFAULT_MAPPING_LAUNCHES)

        default_pkg = (
            self.get_parameter("default_launch_package")
            .get_parameter_value()
            .string_value
            .strip()
        )
        if not default_pkg:
            default_pkg = self.DEFAULT_LAUNCH_PACKAGE
            self.get_logger().warn(
                "Parameter default_launch_package is empty, fallback to robot_bringup."
            )

        common_raw = (
            self.get_parameter("common_launches")
            .get_parameter_value()
            .string_value
        )
        navigation_raw = (
            self.get_parameter("navigation_launches")
            .get_parameter_value()
            .string_value
        )
        mapping_raw = (
            self.get_parameter("mapping_launches")
            .get_parameter_value()
            .string_value
        )

        common_specs: List[LaunchSpec] = []
        mode_specs: Dict[str, List[LaunchSpec]] = {
            "navigation": [],
            "mapping": [],
        }

        common_specs.extend(self._parse_specs_csv("common_launches", common_raw, default_pkg))
        mode_specs["navigation"].extend(
            self._parse_specs_csv("navigation_launches", navigation_raw, default_pkg)
        )
        mode_specs["mapping"].extend(
            self._parse_specs_csv("mapping_launches", mapping_raw, default_pkg)
        )

        self.get_logger().info(
            "Loaded launch specs: common=%d, navigation=%d, mapping=%d"
            % (len(common_specs), len(mode_specs["navigation"]), len(mode_specs["mapping"]))
        )

        return common_specs, mode_specs

    def _parse_specs_csv(self, param_name: str, raw_specs: str, default_pkg: str) -> List[LaunchSpec]:
        """解析 CSV 参数，格式: launch.py 或 package:launch.py。"""

        specs: List[LaunchSpec] = []
        for index, item in enumerate(raw_specs.split(","), start=1):
            entry = item.strip()
            if not entry:
                continue

            package = default_pkg
            launch_file = entry
            if ":" in entry:
                package, launch_file = entry.split(":", 1)
                package = package.strip()
                launch_file = launch_file.strip()

            if not package or not launch_file:
                self.get_logger().warn(
                    f"Invalid entry in {param_name}[{index}]: '{entry}', skip."
                )
                continue

            specs.append(
                LaunchSpec(
                    name=launch_file.replace(".launch.py", ""),
                    package=package,
                    launch_file=launch_file,
                    arguments={},
                )
            )

        return specs

    def _resolve_launch_paths(self, specs: List[LaunchSpec]) -> Dict[str, str]:
        paths: Dict[str, str] = {}
        for spec in specs:
            try:
                pkg_share = get_package_share_directory(spec.package)
            except PackageNotFoundError:
                self.get_logger().error(f"Package {spec.package} not found, skip {spec.name}.")
                continue

            launch_path = os.path.join(pkg_share, "launch", spec.launch_file)
            if not os.path.exists(launch_path):
                self.get_logger().error(f"Launch file {launch_path} not found, skip {spec.name}.")
                continue

            self.get_logger().info(f"Prepared launch {spec.name} -> {launch_path}")
            paths[spec.name] = launch_path
        return paths

    @staticmethod
    def _normalize_mode(mode: str) -> str:
        return mode.strip().lower()

    def _get_specs_for_mode(self, mode: str) -> Optional[List[LaunchSpec]]:
        if mode not in self.mode_specs:
            return None
        return [*self.common_specs, *self.mode_specs[mode]]

    def start_launches(self, mode: Optional[str] = None) -> tuple[bool, str]:
        if self.running:
            return False, "Launches already running"

        target_mode = self.active_mode if mode is None else self._normalize_mode(mode)
        specs = self._get_specs_for_mode(target_mode)
        if not specs:
            return False, f"Unknown or empty mode {target_mode}"

        launch_paths = self._resolve_launch_paths(specs)
        if not launch_paths:
            return False, "No valid launch description to start"

        env = os.environ.copy()
        for spec in specs:
            if spec.name not in launch_paths:
                continue
            cmd = ["ros2", "launch", spec.package, spec.launch_file]
            for key, value in spec.arguments.items():
                cmd.append(f"{key}:={value}")

            pid_file: Optional[Path] = None
            if self.terminal:
                pid_file = self.pid_dir / f"{spec.name}.pid"
                if pid_file.exists():
                    pid_file.unlink()

            wrapped = self._wrap_command(spec.name, cmd, pid_file)
            try:
                proc = subprocess.Popen(wrapped, env=env, start_new_session=True)
            except Exception as exc:
                self.get_logger().error(f"Failed to start {spec.name}: {exc}")
                self.stop_launches()
                return False, f"Failed to start {spec.name}"

            self.processes[spec.name] = proc
            if pid_file is not None:
                self.pid_files[spec.name] = pid_file
            self.get_logger().info(f"Started {spec.name} (PID={proc.pid})")

        self.running = bool(self.processes)
        if not self.running:
            return False, "No launch processes were started"
        self.active_mode = target_mode
        return True, "Launches started"

    def _wrap_command(self, name: str, cmd: List[str], pid_file: Optional[Path]) -> List[str]:
        if not self.terminal:
            return cmd

        script = self._build_terminal_script(cmd, pid_file)
        terminal = self.terminal.lower()

        if "gnome-terminal" in terminal:
            return ["gnome-terminal", "--wait", "--", "bash", "-lc", script]
        if terminal.startswith("xterm"):
            return ["xterm", "-e", "bash", "-lc", script]
        if terminal.startswith("konsole"):
            return ["konsole", "-e", "bash", "-lc", script]
        self.get_logger().warn(
            f"未知终端 {self.terminal}，改用后台进程模式。"
        )
        self.terminal = ""
        return cmd

    @staticmethod
    def _build_terminal_script(cmd: List[str], pid_file: Optional[Path]) -> str:
        joined = " ".join(shlex.quote(part) for part in cmd)
        parts = []
        if pid_file is not None:
            parts.append(f"echo $$ > {shlex.quote(str(pid_file))}")
        parts.append(f"exec {joined}")
        return "; ".join(parts)

    @staticmethod
    def _wait_process(proc: subprocess.Popen, timeout: float) -> bool:
        """封装带超时的 wait，避免无休止阻塞。"""

        if timeout <= 0:
            proc.wait()
            return True

        try:
            proc.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    def stop_launches(self) -> tuple[bool, str]:
        if not self.running and not self.processes:
            return False, "Launches are not running"

        for name, proc in list(self.processes.items()):
            pid_file = self.pid_files.get(name)
            if proc.poll() is not None:
                if pid_file and pid_file.exists():
                    pid_file.unlink(missing_ok=True)
                continue

            self.get_logger().info(f"Stopping {name} (PID={proc.pid})")
            self._signal_child_pid(pid_file, signal.SIGINT)
            child_stopped = self._wait_child_exit(pid_file, 10)
            if child_stopped:
                if not self._wait_process(proc, 5):
                    proc.terminate()
                    self._wait_process(proc, 2)
                continue

            if not self._wait_process(proc, 10):
                self.get_logger().warn(f"{name} did not exit, terminating...")
                self._signal_child_pid(pid_file, signal.SIGTERM)
                proc.terminate()
                if not self._wait_process(proc, 5):
                    self.get_logger().error(f"Force killing {name}")
                    self._signal_child_pid(pid_file, signal.SIGKILL)
                    proc.kill()
                    if not self._wait_process(proc, 2):
                        self.get_logger().error(f"Failed to kill {name}, ignoring")

            if pid_file and pid_file.exists():
                pid_file.unlink(missing_ok=True)
                self.pid_files.pop(name, None)

        self.processes.clear()
        self.pid_files.clear()
        self.running = False
        return True, "Launches stopped"

    def restart_launches(self) -> tuple[bool, str]:
        stop_ok, stop_msg = self.stop_launches() if self.running else (True, "")
        if not stop_ok:
            return False, f"Failed to stop: {stop_msg}"
        return self.start_launches(self.active_mode)

    def switch_mode(self, mode: str) -> tuple[bool, str]:
        target_mode = self._normalize_mode(mode)
        if target_mode not in self.mode_specs:
            return False, f"Unknown mode {target_mode}"
        if self.running and target_mode == self.active_mode:
            return True, f"Already running in {target_mode}"

        stop_ok, stop_msg = self.stop_launches() if self.running else (True, "")
        if not stop_ok:
            return False, f"Failed to stop: {stop_msg}"
        return self.start_launches(target_mode)

    # ==================================================================
    # ROS2 服务回调
    # ==================================================================
    def handle_start(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.start_launches()
        response.success = success
        response.message = message
        return response

    def handle_stop(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.stop_launches()
        response.success = success
        response.message = message
        return response

    def handle_restart(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.restart_launches()
        response.success = success
        response.message = message
        return response

    def handle_start_navigation(
        self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.switch_mode("navigation")
        response.success = success
        response.message = message
        return response

    def handle_start_mapping(
        self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self.switch_mode("mapping")
        response.success = success
        response.message = message
        return response

    def _publish_state(self) -> None:
        msg = String()
        msg.data = "running" if self.running else "stopped"
        self.state_pub.publish(msg)

    def _signal_child_pid(self, pid_file: Optional[Path], sig: int) -> None:
        if pid_file is None or not pid_file.exists():
            return
        pid = self._read_pid(pid_file)
        if pid is None:
            return

        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, sig)
            return
        except ProcessLookupError:
            return
        except PermissionError:
            pass
        except OSError:
            pass

        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().warn(f"Failed to signal child {pid}: {exc}")

    def _wait_child_exit(self, pid_file: Optional[Path], timeout: float) -> bool:
        if pid_file is None or not pid_file.exists():
            return True

        end = time.time() + timeout
        while time.time() < end:
            if not self._child_alive(pid_file):
                return True
            time.sleep(0.2)
        return not self._child_alive(pid_file)

    def _child_alive(self, pid_file: Optional[Path]) -> bool:
        pid = self._read_pid(pid_file)
        if pid is None:
            return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except OSError:
            return True
        return True

    @staticmethod
    def _read_pid(pid_file: Optional[Path]) -> Optional[int]:
        if pid_file is None or not pid_file.exists():
            return None
        try:
            content = pid_file.read_text().strip()
            if not content:
                return None
            return int(content)
        except Exception:
            return None


def main() -> None:
    rclpy.init()
    node = LaunchManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down launches...")
    finally:
        node.stop_launches()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
