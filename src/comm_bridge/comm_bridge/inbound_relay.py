"""
Inbound relay /bridge/cmd/* → /onboard/cmd/*.

Bridges ROS 2 domain 1 (bridge domain shared with the workstation) to domain 0
(NX-internal /onboard/* topics).

Subscriber nodes live on domain 1; publisher nodes live on domain 0.
Each domain runs its own SingleThreadedExecutor in a separate thread.
rcl_publish() is thread-safe, so domain-1 subscriber callbacks may call
publish() on domain-0 publishers directly.

All inbound entries are forced to RELIABLE QoS regardless of yaml value.
A warning is emitted if a yaml entry specifies a different qos.

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

_DOMAIN_BRIDGE = 1   # Bridge domain — /bridge/* topics (shared with PC)
_DOMAIN_ONBOARD = 0  # NX-internal domain — /onboard/* topics

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


def main(args=None) -> None:
    ctx_bridge = Context()
    ctx_onboard = Context()
    rclpy.init(context=ctx_bridge, args=args, domain_id=_DOMAIN_BRIDGE)
    rclpy.init(context=ctx_onboard, args=[], domain_id=_DOMAIN_ONBOARD)

    node_bridge = Node("inbound_relay_bridge", context=ctx_bridge)
    node_onboard = Node("inbound_relay_onboard", context=ctx_onboard)
    logger = node_bridge.get_logger()

    params_path = (
        get_package_share_directory("comm_bridge") + "/config/comm_bridge_params.yaml"
    )

    try:
        relays = _load_relays(params_path)
    except Exception as e:
        logger.error(f"Failed to load relay config: {e}")
        node_bridge.destroy_node()
        node_onboard.destroy_node()
        rclpy.shutdown(context=ctx_bridge)
        rclpy.shutdown(context=ctx_onboard)
        return

    _refs: list = []  # keep sub/pub refs so GC doesn't collect them
    count = 0

    for entry in relays:
        src = entry["src"]
        dst = entry["dst"]
        type_str = entry["type"]
        qos_key = entry.get("qos", "reliable")

        if _is_echo_loop(src, dst):
            logger.error(
                f"Echo-loop detected: {src} → {dst} share same prefix — skipping"
            )
            continue

        if qos_key != "reliable":
            logger.warn(f"Inbound entry {src} has qos={qos_key!r} — forcing reliable")

        try:
            msg_cls = _load_msg_class(type_str)
        except Exception as e:
            logger.error(f"Cannot import {type_str}: {e} — skipping {src}")
            continue

        pub = node_onboard.create_publisher(msg_cls, dst, _RELIABLE_QOS)

        def _make_cb(p):
            def _cb(msg):
                p.publish(msg)
            return _cb

        sub = node_bridge.create_subscription(msg_cls, src, _make_cb(pub), _RELIABLE_QOS)
        _refs.append((sub, pub))
        logger.info(f"relay  {src}  →  {dst}  [reliable]")
        count += 1

    logger.info(
        f"inbound_relay: {count} relay(s) active"
        f" (domain {_DOMAIN_BRIDGE} → {_DOMAIN_ONBOARD})"
    )

    exec_bridge = SingleThreadedExecutor(context=ctx_bridge)
    exec_onboard = SingleThreadedExecutor(context=ctx_onboard)
    exec_bridge.add_node(node_bridge)
    exec_onboard.add_node(node_onboard)

    t_onboard = threading.Thread(target=exec_onboard.spin, daemon=True, name="exec_onboard")
    t_onboard.start()

    try:
        exec_bridge.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec_bridge.shutdown()
        exec_onboard.shutdown()
        t_onboard.join(timeout=5.0)
        node_bridge.destroy_node()
        node_onboard.destroy_node()
        rclpy.shutdown(context=ctx_bridge)
        rclpy.shutdown(context=ctx_onboard)


if __name__ == "__main__":
    main()
