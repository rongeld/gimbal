#!/usr/bin/env python3
"""
Mirror SIYI gimbal attitude to Gazebo JointTrajectory.

Connects to the SIYI gimbal over UDP, requests attitude at a configurable
rate, and publishes joint positions to /set_joint_trajectory.
"""

import argparse
import math
import os
import sys
import threading
import time
import types

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "gimbal_yaw_joint",
    "gimbal_pitch_joint",
    "gimbal_roll_joint",
    "gimbal_3axis_vtol::gimbal_yaw_joint",
    "gimbal_3axis_vtol::gimbal_pitch_joint",
    "gimbal_3axis_vtol::gimbal_roll_joint",
]


def _ensure_simple_pid() -> None:
    try:
        import simple_pid  # noqa: F401
        return
    except Exception:
        pass

    module = types.ModuleType("simple_pid")

    class PID:
        def __init__(self, *args, **kwargs) -> None:
            self.output_limits = (None, None)

        def __call__(self, *_args, **_kwargs) -> float:
            return 0.0

    module.PID = PID
    sys.modules["simple_pid"] = module


def _load_siyi_sdk():
    _ensure_simple_pid()
    try:
        from siyi_sdk import SIYISDK
        return SIYISDK
    except Exception:
        pass

    sdk_path = os.environ.get("SIYI_SDK_PATH")
    if sdk_path:
        sys.path.insert(0, sdk_path)
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.abspath(os.path.join(script_dir, "..", "..", "siyi"))
        if os.path.exists(os.path.join(candidate, "siyi_sdk.py")):
            sys.path.insert(0, candidate)

    from siyi_sdk import SIYISDK

    return SIYISDK


class SiyiAttitudeMirror(Node):
    def __init__(
        self,
        sdk,
        publish_hz: float,
        invert_yaw: bool,
        invert_pitch: bool,
        invert_roll: bool,
    ) -> None:
        super().__init__("siyi_attitude_mirror")
        self._sdk = sdk
        self._invert_yaw = invert_yaw
        self._invert_pitch = invert_pitch
        self._invert_roll = invert_roll
        self._pub = self.create_publisher(JointTrajectory, "/set_joint_trajectory", 10)
        self._period = max(1.0 / publish_hz, 0.001)
        self._timer = self.create_timer(self._period, self._on_timer)
        self._last_seq = -1
        self._last_log = time.time()

        self.get_logger().info(f"Publishing /set_joint_trajectory @ {publish_hz:.1f} Hz")

    def _apply_invert(self, yaw: float, pitch: float, roll: float) -> tuple[float, float, float]:
        if self._invert_yaw:
            yaw = -yaw
        if self._invert_pitch:
            pitch = -pitch
        if self._invert_roll:
            roll = -roll
        return yaw, pitch, roll

    def _on_timer(self) -> None:
        yaw, pitch, roll = self._sdk.getAttitude()
        yaw, pitch, roll = self._apply_invert(yaw, pitch, roll)
        yaw = math.radians(yaw)
        pitch = math.radians(pitch)
        roll = math.radians(roll)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch, roll, yaw, pitch, roll]
        sec = int(self._period)
        nanosec = int((self._period - sec) * 1_000_000_000)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(point)
        self._pub.publish(traj)

        seq = getattr(self._sdk._att_msg, "seq", None)
        if seq is None:
            return
        if seq != self._last_seq:
            self._last_seq = seq
            return

        now = time.time()
        if now - self._last_log >= 1.0:
            self.get_logger().warning("No new gimbal attitude updates yet")
            self._last_log = now


def main() -> None:
    parser = argparse.ArgumentParser(description="Mirror SIYI gimbal attitude to Gazebo")
    parser.add_argument("--ip", default="192.168.144.25", help="SIYI gimbal IP")
    parser.add_argument("--port", type=int, default=37260, help="SIYI gimbal port")
    parser.add_argument("--poll-hz", type=float, default=60.0, help="Gimbal attitude poll rate")
    parser.add_argument("--publish-hz", type=float, default=60.0, help="ROS publish rate")
    parser.add_argument("--invert-yaw", action="store_true", help="Invert yaw")
    parser.add_argument("--invert-pitch", action="store_true", help="Invert pitch")
    parser.add_argument("--invert-roll", action="store_true", help="Invert roll")
    parser.add_argument("--debug", action="store_true", help="Enable SIYI SDK debug logs")
    args, ros_args = parser.parse_known_args()

    try:
        SIYISDK = _load_siyi_sdk()
    except Exception as exc:
        print(f"Failed to import siyi_sdk: {exc}")
        sys.exit(1)

    sdk = SIYISDK(server_ip=args.ip, port=args.port, debug=args.debug)
    sdk._gimbal_att_loop_rate = max(1.0 / args.poll_hz, 0.001)
    sdk._g_att_thread = threading.Thread(
        target=sdk.gimbalAttLoop, args=(sdk._gimbal_att_loop_rate,)
    )
    sdk._g_info_thread = threading.Thread(target=lambda: None)
    sdk._pid_angle_thread = threading.Thread(target=lambda: None)

    if not sdk.connect(maxWaitTime=3.0):
        print("Failed to connect to SIYI gimbal.")
        sys.exit(1)

    rclpy.init(args=ros_args)
    node = SiyiAttitudeMirror(
        sdk,
        args.publish_hz,
        args.invert_yaw,
        args.invert_pitch,
        args.invert_roll,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            sdk.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
