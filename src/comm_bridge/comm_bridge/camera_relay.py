"""
Camera outbound relay /onboard/sensors/camera/* → /bridge/sensors/*
(domain 0 → domain 1).

Separated from outbound_relay so that large-frame DDS publishing is
isolated from the rest of the sensor relay.  Subscription callbacks
return immediately (q.put) and per-publisher daemon threads handle the
actual domain-1 publish asynchronously.  This prevents the relay from
holding FastDDS SHM port resources while serialising 1.8 MB depth frames,
which would otherwise block the realsense2_camera publisher and drop it
below 30 Hz even at low overall CPU utilisation.

Executor: SingleThreadedExecutor — callbacks are just queue puts, no
need for thread-pool overhead here.

Topic table: camera_relay.ros__parameters.relays in comm_bridge_params.yaml.
"""
import importlib
import queue
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_DOMAIN_ONBOARD = 0
_DOMAIN_BRIDGE = 1

_QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _load_msg_class(type_str: str):
    parts = type_str.split("/")
    if len(parts) != 3:
        raise ValueError(f"Invalid message type: {type_str!r}")
    pkg, _sub, name = parts
    return getattr(importlib.import_module(f"{pkg}.{_sub}"), name)


def _load_relays(yaml_path: str) -> list:
    with open(yaml_path) as f:
        doc = yaml.safe_load(f) or {}
    return doc.get("camera_relay", {}).get("ros__parameters", {}).get("relays", [])


def _make_async_cb(pub):
    """Return a subscription callback that returns instantly.

    A daemon thread drains the queue and publishes to domain 1.
    When the thread is busy (publish takes time), stale frames are
    skipped so only the most recent frame is sent — matching the
    BEST_EFFORT + KEEP_LAST + depth=1 QoS semantics.
    """
    q: queue.SimpleQueue = queue.SimpleQueue()

    def _worker():
        while True:
            msg = q.get()
            # Skip any stale frames that queued up while we were publishing
            while True:
                try:
                    msg = q.get_nowait()
                except queue.Empty:
                    break
            pub.publish(msg)

    threading.Thread(target=_worker, daemon=True, name=f"cam_pub_{id(pub)}").start()

    def _cb(msg):
        q.put(msg)

    return _cb


def main(args=None) -> None:
    ctx_onboard = Context()
    ctx_bridge = Context()
    rclpy.init(context=ctx_onboard, args=args, domain_id=_DOMAIN_ONBOARD)
    rclpy.init(context=ctx_bridge, args=[], domain_id=_DOMAIN_BRIDGE)

    node_onboard = Node("camera_relay_onboard", context=ctx_onboard)
    node_bridge = Node("camera_relay_bridge", context=ctx_bridge)
    logger = node_onboard.get_logger()

    params_path = (
        get_package_share_directory("comm_bridge") + "/config/comm_bridge_params.yaml"
    )

    try:
        relays = _load_relays(params_path)
    except Exception as e:
        logger.error(f"Failed to load camera relay config: {e}")
        node_onboard.destroy_node()
        node_bridge.destroy_node()
        rclpy.shutdown(context=ctx_onboard)
        rclpy.shutdown(context=ctx_bridge)
        return

    _refs: list = []
    for entry in relays:
        src = entry["src"]
        dst = entry["dst"]
        type_str = entry["type"]
        try:
            msg_cls = _load_msg_class(type_str)
        except Exception as e:
            logger.error(f"Cannot import {type_str}: {e} — skipping {src}")
            continue

        pub = node_bridge.create_publisher(msg_cls, dst, _QOS_BEST_EFFORT)
        sub = node_onboard.create_subscription(
            msg_cls, src, _make_async_cb(pub), _QOS_BEST_EFFORT)
        _refs.append((sub, pub))
        logger.info(f"camera relay  {src}  →  {dst}")

    logger.info(f"camera_relay: {len(_refs)} relay(s) active")

    exec_bridge = SingleThreadedExecutor(context=ctx_bridge)
    exec_bridge.add_node(node_bridge)
    t_bridge = threading.Thread(
        target=exec_bridge.spin, daemon=True, name="cam_exec_bridge")
    t_bridge.start()

    exec_onboard = SingleThreadedExecutor(context=ctx_onboard)
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
