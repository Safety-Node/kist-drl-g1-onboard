"""goto_node — point-to-point navigator using UWB absolute pose.

Replaces the SLAM/Nav2 stack (REQ-37). The demo environment is fixed and obstacle-free,
so a simple P-controller is sufficient:

    while not at(goal):
        err = goal - uwb_pose
        cmd_vel = clip(Kp * err, ±v_max)
        publish cmd_vel  # → safety_monitor → motor_controller

Inputs:
  /onboard/sensors/uwb/pose   (geometry_msgs/PoseStamped)        — current absolute pose
  /onboard/cmd/nav_goal       (std_msgs/String OR PoseStamped)   — named goal or absolute pose

Outputs:
  /onboard/navigation/cmd_vel (geometry_msgs/Twist)              — to safety_monitor
  /onboard/nav/state          (std_msgs/String)                  — IDLE / MOVING / REACHED / FAILED
"""
import rclpy
from rclpy.node import Node


class GotoNode(Node):
    def __init__(self) -> None:
        super().__init__('goto_node')

        # TODO(REQ-37): declare params (update_rate_hz, kp_linear, kp_angular,
        #               max_linear_speed, max_angular_speed, position_tolerance,
        #               yaw_tolerance, uwb_timeout_s, named_goals_file)
        # TODO(REQ-37): load named_goals.yaml from share/navigation/config/
        # TODO(REQ-37): subscribe /onboard/sensors/uwb/pose (PoseStamped)
        # TODO(REQ-30): subscribe /onboard/cmd/nav_goal (String for named goal, or PoseStamped)
        # TODO(REQ-37): publisher /onboard/navigation/cmd_vel (Twist)
        # TODO(REQ-30): publisher /onboard/nav/state (String) — IDLE / MOVING / REACHED / FAILED
        # TODO(REQ-37): timer at update_rate_hz running the P-controller step

        self.get_logger().info('goto_node started (TBD)')

    # TODO(REQ-37): def _on_uwb_pose(self, msg) -> None  ← cache latest pose + timestamp
    # TODO(REQ-30): def _on_nav_goal(self, msg) -> None  ← resolve named goal, transition to MOVING
    # TODO(REQ-37): def _step(self) -> None              ← P-controller tick, publish cmd_vel
    # TODO(REQ-37): def _at_goal(self) -> bool           ← position + yaw tolerance check
    # TODO(REQ-37): def _halt(self, reason: str) -> None ← publish zero Twist + state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
