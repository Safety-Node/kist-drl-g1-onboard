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

Yaw handling (decided 2026-05-16, navigation review #1):
  UWB hardware gives reliable (x, y) but no yaw, and uwb_node publishes
  identity quaternion to make that explicit. goto_node therefore IGNORES
  yaw on the demo path:
    - kp_angular is set to 0.0 in goto_params.yaml (P-controller emits no
      angular command).
    - yaw_tolerance / NavState.yaw_error_rad / named_goals[*].theta are
      preserved as "reserved" fields so the schema does not change when a
      future IMU-fusion node fills the quaternion. yaw_error_rad reports 0.0.
    - On reaching position tolerance, motor_controller's BalanceStand
      posture decides the final heading (good enough for the named-goal
      kitchen demo).
  Reactivating yaw control is a one-yaml-edit + property remap once a
  fusion node lands; no code in this file needs to change.

Idle cadence (decided 2026-05-16, navigation review #2):
  safety_monitor enforces cmd_vel_timeout_s on /onboard/navigation/cmd_vel.
  goto_node publishes a zero-Twist heartbeat at idle_publish_hz whenever
  state != MOVING so the watchdog stays quiet during IDLE / REACHED /
  FAILED / CANCELED. The heartbeat is harmless downstream (motor_controller
  deadbands near-zero Twists per its own TODO).

Inputs:
- /onboard/sensors/uwb/pose   (PoseStamped)  current absolute pose
- /onboard/cmd/nav_goal       (String)       named goal key (e.g. "refrigerator")

Outputs:
- /onboard/navigation/cmd_vel (Twist)        to safety_monitor
- /onboard/nav/state          (NavState)     IDLE/MOVING/REACHED/FAILED/CANCELED

State machine summary:

         nav_goal known
  IDLE -----------------> MOVING --at_goal--> REACHED
                            |  ^                |
              uwb_stale     |  |                | new nav_goal
                            v  | new nav_goal   v
                          FAILED <----------- (direct)
                            ^                   |
                            +-- new nav_goal -->+
                            |                   |
                          MOVING --new nav_goal-> CANCELED -> MOVING (new)

  Edge meanings:
    * CANCELED is emitted ONLY when a new nav_goal arrives while the
      previous goal is still active (state == MOVING). REACHED and FAILED
      are terminal -- there is no active goal to cancel, so a new
      nav_goal transitions directly to MOVING with no intermediate
      CANCELED message. This keeps the PC trace meaningful:
      "CANCELED" = "we cut a live goal short", not "we replaced a
      finished one".
    * STATUS_FAILED is sticky after uwb_stale -- a fresh nav_goal is
      required to leave it; UWB recovery alone does NOT auto-resume.

  External cancellation: there is no /onboard/cmd/nav_cancel topic by
  design. External cancellation flows through safety_monitor E-STOP.
  See NavState.msg.
"""
import rclpy
from rclpy.node import Node


class GotoNode(Node):
    def __init__(self) -> None:
        super().__init__('goto_node')

        # TODO(REQ-37): declare params (update_rate_hz, idle_publish_hz,
        #               kp_linear, kp_angular, max_linear_speed, max_angular_speed,
        #               position_tolerance, yaw_tolerance, uwb_timeout_s,
        #               named_goals_file)
        # TODO(REQ-30): load named_goals.yaml from share/navigation/config/
        #               via `yaml.safe_load(open(path))` directly, NOT via ROS
        #               parameters. The schema is dict-of-dict
        #               ({name: {x, y, theta}}) which declare_parameter cannot
        #               carry as a single value -- same trap that bit
        #               comm_bridge's relay table. Result: {name: (x, y, theta)}
        #               in-memory dict at startup.
        #               NOTE: theta may be Python None (yaml `null` placeholder
        #               while yaw is RESERVED -- see module docstring). Store
        #               as-is and skip any yaw-related arithmetic while
        #               kp_angular == 0; do not coerce None to 0.0 or the
        #               "not measured yet" signal is lost.
        # TODO(REQ-37): subscribe /onboard/sensors/uwb/pose (geometry_msgs/PoseStamped)
        # TODO(REQ-30, REQ-37): subscribe /onboard/cmd/nav_goal (std_msgs/String)
        # TODO(REQ-37): publisher /onboard/navigation/cmd_vel (geometry_msgs/Twist)
        # TODO(REQ-30): publisher /onboard/nav/state (g1_onboard_msgs/NavState)
        # TODO(REQ-37): main timer at update_rate_hz running _step() when MOVING,
        #               and a separate idle_publish_hz timer publishing zero-Twist
        #               whenever state != MOVING (safety_monitor watchdog feed).

        self.get_logger().info('goto_node started (TBD)')

    # TODO(REQ-37): def _on_uwb_pose(self, msg)  -- cache latest pose + timestamp
    # TODO(REQ-30): def _on_nav_goal(self, msg)
    #               unknown name -> publish NavState(FAILED, "unknown goal '<name>'");
    #               known name -> store (x, y, theta) and transition by current state:
    #                 from MOVING (preemption)  : emit NavState(CANCELED,
    #                                             "preempted by '<new>'") for the
    #                                             OLD goal, THEN NavState(MOVING, ...)
    #                                             for the new one -- two messages,
    #                                             in that order, so the PC trace
    #                                             stays continuous.
    #                 from IDLE / REACHED / FAILED : emit NavState(MOVING, ...)
    #                                             directly. There is no active
    #                                             goal to cancel; REACHED and
    #                                             FAILED are terminal states.
    # TODO(REQ-37): def _step(self)
    #               1. if (now - last_uwb_stamp) > uwb_timeout_s:
    #                       _halt("uwb_timeout") -- emit FAILED, NOT auto-resume.
    #                       New nav_goal is required after UWB recovers; the
    #                       PC must explicitly retry. Avoids "surprising resume"
    #                       in the PC trace.
    #               2. else: P-controller tick.
    #                       cmd_vel.linear.x  = clip(kp_linear  * err.x, +/- max_linear_speed)
    #                       cmd_vel.linear.y  = clip(kp_linear  * err.y, +/- max_linear_speed)
    #                       cmd_vel.angular.z = 0.0   (yaw drop -- review #1)
    #                       Per-axis clamp (NOT magnitude clamp) means the
    #                       combined speed on a diagonal is up to
    #                       sqrt(2) * max_linear_speed; still within
    #                       safety_monitor's max_linear_x/y limits which are
    #                       enforced per-axis as well. Acceptable for demo.
    # TODO(REQ-37): def _at_goal(self) -> bool
    #               Position tolerance only. yaw_tolerance is reserved and
    #               currently ignored (see module docstring).
    # TODO(REQ-37): def _halt(self, reason)
    #               Publish zero-Twist immediately + NavState (FAILED with
    #               `error_message=reason`). The idle timer then keeps the
    #               heartbeat alive.
    # TODO(REQ-37): def _publish_idle_heartbeat(self)
    #               Called from the idle timer at idle_publish_hz whenever
    #               state != MOVING. Publishes zero Twist so safety_monitor's
    #               cmd_vel_timeout_s watchdog stays quiet.


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
