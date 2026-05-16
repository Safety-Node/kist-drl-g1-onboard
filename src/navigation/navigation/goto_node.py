"""
Point-to-point navigator using UWB absolute pose.

Inputs:
- /onboard/sensors/uwb/pose   (PoseStamped)  current absolute pose
- /onboard/cmd/nav_goal       (String)       named goal key (e.g. "refrigerator")

Outputs:
- /onboard/navigation/cmd_vel (Twist)        → safety_monitor
- /onboard/nav/state          (NavState)     IDLE/MOVING/REACHED/FAILED/CANCELED

State machine:
  IDLE / REACHED / FAILED  ── new nav_goal ──→  MOVING (no CANCELED — nothing live)
  MOVING                   ── new nav_goal ──→  CANCELED(old) → MOVING(new)
  MOVING                   ── at_goal ────────→  REACHED
  MOVING                   ── uwb_stale ──────→  FAILED (sticky; UWB recovery does NOT auto-resume)

  External cancel: no /onboard/cmd/nav_cancel by design — use safety_monitor E-STOP.

Traps:
- named_goals.yaml is dict-of-dict → load via yaml.safe_load directly, NOT
  via declare_parameter (would flatten the schema).
- theta may be Python `None` (yaml `null` while yaw is reserved). Do NOT
  coerce to 0.0 — placeholder vs surveyed-0.0 distinction is lost otherwise.
- yaw is dropped: kp_angular=0 in yaml; angular cmd never emitted; reserved
  for future IMU-fusion node.
- safety_monitor has cmd_vel_timeout_s=2.0; this node must publish a
  zero-Twist heartbeat at idle_publish_hz when state != MOVING.
- update_rate_hz and idle_publish_hz are both float (DoubleType) — rclpy
  parameter overrides must match.

TODO(REQ-37) [TASK-35]: declare params + load named_goals.yaml via yaml.safe_load.
TODO(REQ-37) [TASK-35]: subscribers + publishers wired; main timer + idle heartbeat timer.
TODO(REQ-37) [TASK-35]: P-controller _step with UWB staleness halt + per-axis clamp.
TODO(REQ-30) [TASK-35]: nav_goal lookup; unknown → FAILED; preempt MOVING → CANCELED+MOVING.
TODO(REQ-37) [TASK-35]: _halt(reason) + _publish_idle_heartbeat at idle_publish_hz.
"""
import rclpy
from rclpy.node import Node


class GotoNode(Node):
    def __init__(self) -> None:
        super().__init__('goto_node')
        # TODO(REQ-37) [TASK-35]: wire everything (see module docstring TODO list).
        self.get_logger().info('goto_node started (TBD)')


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
