"""
Inbound relay /bridge/cmd/* → /onboard/cmd/*.

Loads comm_bridge_params.yaml directly from the package share directory
(yaml.safe_load). List-of-dict relay entries cannot be expressed as ROS 2
parameters (rclpy raises InvalidParameterTypeException), so NO parameters=[]
in the launch file — see comm_bridge.launch.py.

All inbound entries are forced to RELIABLE QoS regardless of yaml value.
A warning is emitted if a yaml entry specifies a different qos.

Echo-loop guard: entries where src and dst share the same namespace prefix
(both /onboard/ or both /bridge/) are rejected at load time.
"""
import importlib

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _load_msg_class(type_str: str):
    """'g1_onboard_msgs/msg/JointCmd' → g1_onboard_msgs.msg.JointCmd class."""
    parts = type_str.split("/")
    if len(parts) != 3:
        raise ValueError(f"Invalid message type: {type_str!r} (expected pkg/msg/Name)")
    pkg, _sub, name = parts
    mod = importlib.import_module(f"{pkg}.{_sub}")
    return getattr(mod, name)


def _load_relays(yaml_path: str) -> list:
    with open(yaml_path) as f:
        doc = yaml.safe_load(f) or {}
    return doc.get("inbound_relay", {}).get("ros__parameters", {}).get("relays", [])


def _is_echo_loop(src: str, dst: str) -> bool:
    """Return True if src and dst share the same top-level namespace prefix."""
    for prefix in ("/onboard/", "/bridge/"):
        if src.startswith(prefix) and dst.startswith(prefix):
            return True
    return False


class InboundRelay(Node):
    def __init__(self) -> None:
        super().__init__("inbound_relay")

        params_path = (
            get_package_share_directory("comm_bridge") + "/config/comm_bridge_params.yaml"
        )

        try:
            relays = _load_relays(params_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load relay config: {e}")
            return

        self._pairs: list = []  # keep refs so GC doesn't collect sub/pub

        for entry in relays:
            src = entry["src"]
            dst = entry["dst"]
            type_str = entry["type"]
            qos_key = entry.get("qos", "reliable")

            if _is_echo_loop(src, dst):
                self.get_logger().error(
                    f"Echo-loop detected: {src} → {dst} share same prefix — skipping"
                )
                continue

            if qos_key != "reliable":
                self.get_logger().warn(
                    f"Inbound entry {src} has qos={qos_key!r} — forcing reliable"
                )

            try:
                msg_cls = _load_msg_class(type_str)
            except Exception as e:
                self.get_logger().error(f"Cannot import {type_str}: {e} — skipping {src}")
                continue

            pub = self.create_publisher(msg_cls, dst, _RELIABLE_QOS)

            def _make_cb(p):
                def _cb(msg):
                    p.publish(msg)
                return _cb

            sub = self.create_subscription(msg_cls, src, _make_cb(pub), _RELIABLE_QOS)
            self._pairs.append((sub, pub))
            self.get_logger().info(f"relay  {src}  →  {dst}  [reliable]")

        self.get_logger().info(f"inbound_relay: {len(self._pairs)} relay(s) active")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InboundRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
