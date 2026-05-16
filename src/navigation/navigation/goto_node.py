"""
Point-to-point navigator using UWB absolute pose.

The PC publishes the target as a human-readable string (e.g. "refrigerator")
on /bridge/cmd/nav_goal, comm_bridge relays it to /onboard/cmd/nav_goal as
std_msgs/String, and goto_node looks the name up in named_goals.yaml to
resolve it into (x, y, theta) in the UWB 'map' frame. Unknown names ->
NavState.STATUS_FAILED with an explanatory message.

This is a 2026-05-16 revert of decision 5a: keeping the named-goal table on
NX (instead of resolving on PC) trades a bit of NX-side coupling for much
easier debugging -- a developer can `ros2 topic pub /bridge/cmd/nav_goal
std_msgs/String "data: 'refrigerator'"` from any machine without first
setting up the full PC stack.

The demo environment is fixed and obstacle-free, so a simple P-controller
is sufficient:

    while not at(goal):
        err = goal - uwb_pose
        cmd_vel = clip(Kp * err, +/- v_max)
        publish cmd_vel  # -> safety_monitor -> motor_controller

Inputs:
- /onboard/sensors/uwb/pose   (PoseStamped)  current absolute pose
- /onboard/cmd/nav_goal       (String)       named goal key (e.g. "refrigerator")

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
        #               yaw_tolerance, uwb_timeout_s, named_goals_file)
        # TODO(REQ-30): load named_goals.yaml from share/navigation/config/
        #               into a dict {name: (x, y, theta)} at startup
        # TODO(REQ-37): subscribe /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped)
        # TODO(REQ-30, REQ-37): subscribe /onboard/cmd/nav_goal (std_msgs/String)
        # TODO(REQ-37): publisher /onboard/navigation/cmd_vel (geometry_msgs/Twist)
        # TODO(REQ-30): publisher /onboard/nav/state (g1_onboard_msgs/NavState)
        # TODO(REQ-37): timer at update_rate_hz running the P-controller step

        self.get_logger().info('goto_node started (TBD)')

    # TODO(REQ-37): def _on_uwb_pose(self, msg)  -- cache latest pose + timestamp
    # TODO(REQ-30): def _on_nav_goal(self, msg)  -- look up msg.data in named_goals;
    #               unknown name -> publish NavState(FAILED, "unknown goal '<name>'");
    #               known name -> store (x, y, theta), transition to MOVING (preempts)
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
