"""
Outbound relay /onboard/* → /bridge/*.

Loads comm_bridge_params.yaml directly from the package share directory
(yaml.safe_load). List-of-dict relay entries cannot be expressed as ROS 2
parameters (rclpy raises InvalidParameterTypeException), so NO parameters=[]
in the launch file — see comm_bridge.launch.py.

Each relay entry (src, dst, type, qos) creates one subscriber + one publisher.
Message types are imported dynamically: "sensor_msgs/msg/Imu" becomes
  from sensor_msgs.msg import Imu

Executor: MultiThreadedExecutor so that large-message callbacks (e.g. 1.8 MB
depth images at 30 Hz) do not block audio/IMU callbacks on the same node.
Without this, a depth-image callback holding the GIL for ~10-30 ms causes the
depth=1 subscription queue for audio_pcm (50 Hz, 640 B) to overflow and drop
roughly half the frames before the executor gets back to it.

QoS "stream": BEST_EFFORT + depth=10 for continuous audio — prevents DDS from
discarding frames during brief executor scheduling jitter.  Large-message sensor
streams (depth, color) keep depth=1 ("freshness wins").
"""
import importlib

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

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


class OutboundRelay(Node):
    def __init__(self) -> None:
        super().__init__("outbound_relay")

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
            qos_key = entry.get("qos", "best_effort")

            try:
                msg_cls = _load_msg_class(type_str)
            except Exception as e:
                self.get_logger().error(f"Cannot import {type_str}: {e} — skipping {src}")
                continue

            qos = _QOS_MAP.get(qos_key)
            if qos is None:
                self.get_logger().warn(
                    f"Unknown qos {qos_key!r} for {src} — falling back to best_effort"
                )
                qos = _QOS_MAP["best_effort"]

            pub = self.create_publisher(msg_cls, dst, qos)

            def _make_cb(p):
                def _cb(msg):
                    p.publish(msg)
                return _cb

            sub = self.create_subscription(msg_cls, src, _make_cb(pub), qos)
            self._pairs.append((sub, pub))
            self.get_logger().info(f"relay  {src}  →  {dst}  [{qos_key}]")

        self.get_logger().info(f"outbound_relay: {len(self._pairs)} relay(s) active")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OutboundRelay()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
