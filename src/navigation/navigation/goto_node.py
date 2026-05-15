"""
Point-to-point navigator using UWB absolute pose.

Per the 2026-05-15 comm_bridge review (decision 5a), named goals
(e.g. "refrigerator", "sink") are resolved to absolute coordinates on
the PC side (UnitreeG1 Provider / Move Connector) before being published.
goto_node only accepts PoseStamped goals -- keeping NX dumb and PC smart
simplifies updates to the demo target list (only PC has to redeploy
when the survey changes).

The demo environment is fixed and obstacle-free, so a simple P-controller
is sufficient:

    while not at(goal):
        err = goal - uwb_pose
        cmd_vel = clip(Kp * err, +/- v_max)
        publish cmd_vel  # -> safety_monitor -> motor_controller

Inputs:
- /onboard/sensors/uwb/pose   (PoseStamped)  current absolute pose
- /onboard/cmd/nav_goal       (PoseStamped)  target pose in map frame

Outputs:
- /onboard/navigation/cmd_vel (Twist)        to safety_monitor
- /onboard/nav/state          (NavState)     IDLE/MOVING/REACHED/FAILED/CANCELED
"""
import rclpy
from rclpy.node import Node


class GotoNode(Node):
    def __init__(self) -> None:
        super().__init__('goto_node')

        # TODO(REQ-37): declare params (update_rate_hz, kp_linear, kp_angular,
        #               max_linear_speed, max_angular_speed, position_tolerance,
        #               yaw_tolerance, uwb_timeout_s)
        # TODO(REQ-37): subscribe /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped)
        # TODO(REQ-30, REQ-37): subscribe /onboard/cmd/nav_goal (geometry_msgs/PoseStamped)
        # TODO(REQ-37): publisher /onboard/navigation/cmd_vel (geometry_msgs/Twist)
        # TODO(REQ-30): publisher /onboard/nav/state (kist_drl_g1_msgs/NavState)
        # TODO(REQ-37): timer at update_rate_hz running the P-controller step

        self.get_logger().info('goto_node started (TBD)')

    # TODO(REQ-37): def _on_uwb_pose(self, msg)  -- cache latest pose + timestamp
    # TODO(REQ-30): def _on_nav_goal(self, msg)  -- accept new target (preempts current
    #               MOVING goal); transition to MOVING
    # TODO(REQ-37): def _step(self)              -- P-controller tick, publish cmd_vel
    # TODO(REQ-37): def _at_goal(self) -> bool   -- position + yaw tolerance check
    # TODO(REQ-37): def _halt(self, reason)      -- publish zero Twist + NavState


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
