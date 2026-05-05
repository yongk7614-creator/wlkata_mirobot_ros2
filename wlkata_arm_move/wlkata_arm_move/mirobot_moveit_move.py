import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import serial
from control_msgs.action import FollowJointTrajectory


class MirobotGcodeDriver(Node):

    _OK               = FollowJointTrajectory.Result.SUCCESSFUL
    _ERR_INVALID_GOAL = FollowJointTrajectory.Result.INVALID_GOAL
    _ERR_INVALID_JTS  = FollowJointTrajectory.Result.INVALID_JOINTS
    _ERR_PATH_TOL     = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
    _ERR_GOAL_TOL     = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED

    def __init__(self):
        super().__init__("mirobot_gcode_driver")

        # ── 파라미터 선언 ──────────────────────────────────────────────
        self.declare_parameter(
            "action_name",
            "mirobot_group_controller/follow_joint_trajectory",
        )
        self.declare_parameter("serial_port",        "/dev/ttyUSB0")
        self.declare_parameter("baud_rate",          115200)
        self.declare_parameter("serial_timeout_sec", 1.0)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("command_prefix",    "M21 G90 G00")
        self.declare_parameter("line_ending",       "\r\n")
        self.declare_parameter("wait_for_reply",    True)
        self.declare_parameter("auto_home",         False)
        self.declare_parameter("startup_delay_sec", 2.0)
        self.declare_parameter("settle_delay_sec",  0.0)

        self.action_name    = str(self.get_parameter("action_name").value)
        self.serial_port    = str(self.get_parameter("serial_port").value)
        self.baud_rate      = int(self.get_parameter("baud_rate").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout_sec").value)
        self.joint_names    = list(self.get_parameter("joint_names").value)
        self.cmd_prefix     = str(self.get_parameter("command_prefix").value).strip()
        self.line_ending    = str(self.get_parameter("line_ending").value)
        self.wait_reply     = bool(self.get_parameter("wait_for_reply").value)
        self.auto_home      = bool(self.get_parameter("auto_home").value)
        self.startup_delay  = float(self.get_parameter("startup_delay_sec").value)
        self.settle_delay   = float(self.get_parameter("settle_delay_sec").value)

        self.get_logger().info(
            "Parameters | action=%s  serial=%s@%d  joints=%s  auto_home=%s"
            % (
                self.action_name,
                self.serial_port,
                self.baud_rate,
                self.joint_names,
                self.auto_home,
            )
        )

    
        self._executing = False
        self._exec_lock = threading.Lock()

        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.serial_timeout,
            )
            self.get_logger().info(
                "Serial opened: %s @ %d baud" % (self.serial_port, self.baud_rate)
            )
        except serial.SerialException as exc:
            self.get_logger().error(
                "Failed to open serial port %s: %s" % (self.serial_port, str(exc))
            )
            raise

        if self.startup_delay > 0.0:
            self.get_logger().info(
                "Waiting %.1fs for Mirobot to boot..." % self.startup_delay
            )
            time.sleep(self.startup_delay)

        if self.auto_home:
            self.get_logger().info("auto_home=True: sending homing command...")
            self._send_raw("o105=8")
            self.get_logger().info("Homing command sent.")
        else:
            self.get_logger().info(
                "auto_home=False: homing skipped."
            )

        # ── FollowJointTrajectory 액션 서버 ───────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "Action server ready: /%s" % self.action_name
        )


    def _goal_cb(self, goal_request):
        traj = goal_request.trajectory

        if not traj.joint_names:
            self.get_logger().error("Goal rejected: trajectory has no joint_names.")
            return GoalResponse.REJECT

        if not traj.points:
            self.get_logger().error("Goal rejected: trajectory has no points.")
            return GoalResponse.REJECT

        missing = [j for j in self.joint_names if j not in traj.joint_names]
        if missing:
            self.get_logger().error(
                "Goal rejected: joints %s not found in trajectory." % missing
            )
            return GoalResponse.REJECT

        with self._exec_lock:
            if self._executing:
                self.get_logger().warn(
                    "Goal rejected: already executing a trajectory. "
                    "Wait for current execution to finish."
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            "Goal accepted: %d waypoints | joints=%s"
            % (len(traj.points), list(traj.joint_names))
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel request received.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        result = FollowJointTrajectory.Result()

        with self._exec_lock:
            self._executing = True

        try:
            return self._run_trajectory(goal_handle, result)
        finally:
            with self._exec_lock:
                self._executing = False

    def _run_trajectory(self, goal_handle, result):
        traj = goal_handle.request.trajectory

        try:
            idx_map = self._build_index_map(traj.joint_names)
        except ValueError as exc:
            self.get_logger().error("Joint mapping error: %s" % str(exc))
            result.error_code   = self._ERR_INVALID_JTS
            result.error_string = str(exc)
            goal_handle.abort()
            return result

        n = len(traj.points)
        self.get_logger().info("Executing trajectory: %d waypoints." % n)

       last_t = (
            traj.points[-1].time_from_start.sec
            + traj.points[-1].time_from_start.nanosec * 1e-9
        )
        has_timing = last_t > 0.0

        exec_start = time.monotonic()

        for i, point in enumerate(traj.points):

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Trajectory cancelled at point %d." % i)
                result.error_code   = self._ERR_PATH_TOL
                result.error_string = "Cancelled at point %d." % i
                goal_handle.canceled()
                return result

            try:
                positions_rad = [
                    point.positions[idx_map[name]] for name in self.joint_names
                ]
            except (IndexError, KeyError) as exc:
                self.get_logger().error(
                    "Position extraction failed at point %d: %s" % (i, str(exc))
                )
                result.error_code   = self._ERR_INVALID_GOAL
                result.error_string = "Position error at point %d." % i
                goal_handle.abort()
                return result

             t_target = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )
            if has_timing and t_target > 0.0:
                t_elapsed = time.monotonic() - exec_start
                t_wait    = t_target - t_elapsed
                if t_wait > 0.0:
                    time.sleep(t_wait)

            gcode = self._build_gcode(positions_rad)
            self.get_logger().info(
                "[%d/%d] t=%.3fs | %s" % (i + 1, n, t_target, gcode.strip())
            )

            ok, reply = self._send_command(gcode)
            if not ok:
                self.get_logger().error("Serial error at point %d." % i)
                result.error_code   = self._ERR_GOAL_TOL
                result.error_string = "Serial error at point %d." % i
                goal_handle.abort()
                return result

            if reply:
                self.get_logger().debug("Mirobot: %s" % reply)

            if not has_timing and self.settle_delay > 0.0:
                time.sleep(self.settle_delay)

        self.get_logger().info(
            "Trajectory complete: %d waypoints sent." % n
        )
        print("___________________________________")

        result.error_code   = self._OK
        result.error_string = ""
        goal_handle.succeed()
        return result

    def _build_index_map(self, received_names) -> dict:
        received = list(received_names)
        mapping  = {}
        for name in self.joint_names:
            if name not in received:
                raise ValueError(
                    "Joint '%s' not found in trajectory joint_names=%s"
                    % (name, received)
                )
            mapping[name] = received.index(name)
        return mapping

    def _build_gcode(self, positions_rad: list) -> str:
        degs = [round(math.degrees(r), 2) for r in positions_rad]
        j1, j2, j3, j4, j5, j6 = degs
        return (
            "%s X%.2f Y%.2f Z%.2f A%.2f B%.2f C%.2f%s"
            % (self.cmd_prefix, j1, j2, j3, j4, j5, j6, self.line_ending)
        )

    def _send_command(self, gcode: str) -> tuple:
        try:
            self.ser.write(gcode.encode("utf-8"))
            if self.wait_reply:
                raw   = self.ser.readline()
                reply = raw.decode("utf-8", errors="replace").strip()
            else:
                reply = ""
            return True, reply
        except Exception as exc:
            self.get_logger().error("Serial exception: %s" % str(exc))
            return False, ""

    def _send_raw(self, cmd: str) -> None:
        self.ser.write((cmd + self.line_ending).encode("utf-8"))
        if self.wait_reply:
            self.ser.readline()

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main():
    rclpy.init()
    node = MirobotGcodeDriver()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
