"""
Outbound relay /onboard/* → /bridge/*.

Bridges ROS 2 domain 0 (NX-internal /onboard/* topics) to domain 1 (bridge
domain shared with the workstation /bridge/* topics).

Subscriber nodes live on domain 0; publisher nodes live on domain 1.
Each domain runs its own SingleThreadedExecutor in a separate thread so that
neither blocks the other.  rcl_publish() is thread-safe, so subscriber
callbacks on the domain-0 thread may call publish() on domain-1 publishers
directly without a queue.

Loads comm_bridge_params.yaml directly from the package share directory
(yaml.safe_load). List-of-dict relay entries cannot be expressed as ROS 2
parameters (rclpy raises InvalidParameterTypeException), so NO parameters=[]
in the launch file — see comm_bridge.launch.py.

Echo-loop guard: entries where src and dst share the same namespace prefix
(both /onboard/ or both /bridge/) are rejected at load time.
"""
import importlib
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_DOMAIN_ONBOARD = 0  # NX-internal domain — /onboard/* topics
_DOMAIN_BRIDGE = 1   # Bridge domain — /bridge/* topics (shared with PC)

_QOS_MAP = {
    "best_effort": QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    ),
    "reliable": QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    ),
}


def _load_msg_class(type_str: str):
    """'sensor_msgs/msg/Imu' → sensor_msgs.msg.Imu class."""
    parts = type_str.split("/")
    if len(parts) != 3:
        raise ValueError(f"Invalid message type: {type_str!r} (expected pkg/msg/Name)")
    pkg, _sub, name = parts
    mod = importlib.import_module(f"{pkg}.{_sub}")
    return getattr(mod, name)


def _load_relays(yaml_path: str) -> list:
    with open(yaml_path) as f:
        doc = yaml.safe_load(f) or {}
    return doc.get("outbound_relay", {}).get("ros__parameters", {}).get("relays", [])


def _is_echo_loop(src: str, dst: str) -> bool:
    """Return True if src and dst share the same top-level namespace prefix."""
    for prefix in ("/onboard/", "/bridge/"):
        if src.startswith(prefix) and dst.startswith(prefix):
            return True
    return False


def main(args=None) -> None:
    ctx_onboard = Context()
    ctx_bridge = Context()
    rclpy.init(context=ctx_onboard, args=args, domain_id=_DOMAIN_ONBOARD)
    rclpy.init(context=ctx_bridge, args=[], domain_id=_DOMAIN_BRIDGE)

    node_onboard = Node("outbound_relay_onboard", context=ctx_onboard)
    node_bridge = Node("outbound_relay_bridge", context=ctx_bridge)
    logger = node_onboard.get_logger()

    params_path = (
        get_package_share_directory("comm_bridge") + "/config/comm_bridge_params.yaml"
    )

    try:
        relays = _load_relays(params_path)
    except Exception as e:
        logger.error(f"Failed to load relay config: {e}")
        node_onboard.destroy_node()
        node_bridge.destroy_node()
        rclpy.shutdown(context=ctx_onboard)
        rclpy.shutdown(context=ctx_bridge)
        return

    _refs: list = []  # keep sub/pub refs so GC doesn't collect them
    count = 0

    for entry in relays:
        src = entry["src"]
        dst = entry["dst"]
        type_str = entry["type"]
        qos_key = entry.get("qos", "best_effort")

        if _is_echo_loop(src, dst):
            logger.error(
                f"Echo-loop detected: {src} → {dst} share same prefix — skipping"
            )
            continue

        try:
            msg_cls = _load_msg_class(type_str)
        except Exception as e:
            logger.error(f"Cannot import {type_str}: {e} — skipping {src}")
            continue

        qos = _QOS_MAP.get(qos_key)
        if qos is None:
            logger.warn(f"Unknown qos {qos_key!r} for {src} — falling back to best_effort")
            qos = _QOS_MAP["best_effort"]

        pub = node_bridge.create_publisher(msg_cls, dst, qos)

        def _make_cb(p):
            def _cb(msg):
                p.publish(msg)
            return _cb

        sub = node_onboard.create_subscription(msg_cls, src, _make_cb(pub), qos)
        _refs.append((sub, pub))
        logger.info(f"relay  {src}  →  {dst}  [{qos_key}]")
        count += 1

    logger.info(
        f"outbound_relay: {count} relay(s) active"
        f" (domain {_DOMAIN_ONBOARD} → {_DOMAIN_BRIDGE})"
    )

    exec_onboard = SingleThreadedExecutor(context=ctx_onboard)
    exec_bridge = SingleThreadedExecutor(context=ctx_bridge)
    exec_onboard.add_node(node_onboard)
    exec_bridge.add_node(node_bridge)

    t_bridge = threading.Thread(target=exec_bridge.spin, daemon=True, name="exec_bridge")
    t_bridge.start()

    try:
        exec_onboard.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec_onboard.shutdown()
        exec_bridge.shutdown()
        t_bridge.join(timeout=5.0)
        node_onboard.destroy_node()
        node_bridge.destroy_node()
        rclpy.shutdown(context=ctx_onboard)
        rclpy.shutdown(context=ctx_bridge)


if __name__ == "__main__":
    main()
