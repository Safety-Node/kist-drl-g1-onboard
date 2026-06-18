"""
Outbound relay /onboard/* → /bridge/*  (domain 0 → domain 1).

Domain layout:
  domain 0 — NX-internal: all /onboard/* topics (sensors, motor, safety …)
  domain 1 — Bridge domain: shared with workstation /bridge/* topics

Each relay entry subscribes on domain 0 and publishes on domain 1 so the
workstation can see sensor data without being on the robot's internal domain.

Executor: MultiThreadedExecutor on the domain-0 node so that large-message
callbacks (e.g. 1.8 MB depth images at 30 Hz) do not block audio/IMU callbacks.
Without this, a depth callback holding the executor for ~10-30 ms causes the
depth=1 subscription queue for audio_pcm (50 Hz, 640 B) to overflow and drop
roughly half the frames before the executor gets back to it.

QoS "stream": BEST_EFFORT + depth=10 on the domain-0 subscriber (onboard-local,
low-loss IPC). The domain-1 publisher (network-facing) defaults to the same QoS
unless overridden by "qos_pub" in the yaml entry. Use qos_pub: reliable for
continuous streams (e.g. audio_pcm) that must survive WiFi/Ethernet packet loss —
BEST_EFFORT over UDP gives no retransmission, causing intermittent frame gaps.

Large-image topics keep depth=1 ("freshness wins").

Loads comm_bridge_params.yaml directly from the package share directory
(yaml.safe_load). List-of-dict relay entries cannot be expressed as ROS 2
parameters (rclpy raises InvalidParameterTypeException), so NO parameters=[]
in the launch file — see comm_bridge.launch.py.
"""
import importlib
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_DOMAIN_ONBOARD = 0  # NX-internal — /onboard/* topics
_DOMAIN_BRIDGE = 1   # shared with workstation — /bridge/* topics

_QOS_MAP = {
    # Large sensor frames: always want the latest, never accumulate stale data.
    "best_effort": QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    ),
    # Continuous audio stream: needs delivery continuity, not just freshness.
    # depth=10 absorbs brief executor-scheduling jitter without accumulating
    # stale frames (audio at 50 Hz drains the queue in 200 ms even when full).
    "stream": QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
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


def main(args=None) -> None:
    ctx_onboard = Context()
    ctx_bridge = Context()
    rclpy.init(context=ctx_onboard, args=args, domain_id=_DOMAIN_ONBOARD)
    rclpy.init(context=ctx_bridge, args=[], domain_id=_DOMAIN_BRIDGE)

    # domain-0 node: subscribes /onboard/* topics
    node_onboard = Node("outbound_relay_onboard", context=ctx_onboard)
    # domain-1 node: publishes /bridge/* topics visible to workstation
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
        qos_sub_key = entry.get("qos", "best_effort")
        qos_pub_key = entry.get("qos_pub", qos_sub_key)

        try:
            msg_cls = _load_msg_class(type_str)
        except Exception as e:
            logger.error(f"Cannot import {type_str}: {e} — skipping {src}")
            continue

        qos_sub = _QOS_MAP.get(qos_sub_key)
        if qos_sub is None:
            logger.warn(f"Unknown qos {qos_sub_key!r} for {src} — falling back to best_effort")
            qos_sub = _QOS_MAP["best_effort"]

        qos_pub = _QOS_MAP.get(qos_pub_key)
        if qos_pub is None:
            logger.warn(f"Unknown qos_pub {qos_pub_key!r} for {dst} — falling back to best_effort")
            qos_pub = _QOS_MAP["best_effort"]

        # publisher lives on domain 1 (workstation-visible, network-facing)
        pub = node_bridge.create_publisher(msg_cls, dst, qos_pub)

        def _make_cb(p):
            def _cb(msg):
                p.publish(msg)
            return _cb

        # subscriber lives on domain 0 (onboard-internal)
        sub = node_onboard.create_subscription(msg_cls, src, _make_cb(pub), qos_sub)
        _refs.append((sub, pub))
        if qos_pub_key != qos_sub_key:
            logger.info(f"relay  {src}  →  {dst}  [sub={qos_sub_key} pub={qos_pub_key}]")
        else:
            logger.info(f"relay  {src}  →  {dst}  [{qos_sub_key}]")
        count += 1

    logger.info(
        f"outbound_relay: {count} relay(s) active"
        f" (domain {_DOMAIN_ONBOARD} → {_DOMAIN_BRIDGE})"
    )

    # domain-1 publisher node runs in a background thread (publish-only, lightweight)
    exec_bridge = MultiThreadedExecutor(context=ctx_bridge)
    exec_bridge.add_node(node_bridge)
    t_bridge = threading.Thread(target=exec_bridge.spin, daemon=True, name="exec_bridge")
    t_bridge.start()

    # domain-0 subscriber node uses MultiThreadedExecutor so depth-image callbacks
    # don't block audio/IMU callbacks
    exec_onboard = MultiThreadedExecutor(context=ctx_onboard)
    exec_onboard.add_node(node_onboard)
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
