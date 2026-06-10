/**
 * camera_relay — C++ domain bridge for camera frames.
 *
 * Relays camera topics from domain 0 (onboard) to domain 1 (bridge).
 * C++ serialization handles 1.8 MB depth frames at 30 Hz without the
 * ~125 ms Python/rclpy CDR overhead that saturates a CPU core.
 *
 * Subscriptions  (domain 0, BEST_EFFORT depth=1):
 *   /onboard/sensors/camera/color/image_raw/compressed
 *   /onboard/sensors/camera/aligned_depth_to_color/image_raw
 *
 * Publications   (domain 1, BEST_EFFORT depth=1):
 *   /bridge/sensors/color/compressed
 *   /bridge/sensors/depth/image_raw
 */
#include <atomic>
#include <csignal>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr size_t kDomainOnboard = 0;
constexpr size_t kDomainBridge  = 1;

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop.store(true); }

rclcpp::QoS camera_qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

}  // namespace

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // --- domain 0: onboard subscriber context ---
    auto ctx_onboard = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_onboard;
    init_onboard.set_domain_id(kDomainOnboard);
    ctx_onboard->init(argc, argv, init_onboard);

    // --- domain 1: bridge publisher context ---
    auto ctx_bridge = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_bridge;
    init_bridge.set_domain_id(kDomainBridge);
    ctx_bridge->init(argc, argv, init_bridge);

    rclcpp::NodeOptions opts_sub;
    opts_sub.context(ctx_onboard);
    auto node_sub = std::make_shared<rclcpp::Node>("camera_relay_sub", opts_sub);

    rclcpp::NodeOptions opts_pub;
    opts_pub.context(ctx_bridge);
    auto node_pub = std::make_shared<rclcpp::Node>("camera_relay_pub", opts_pub);

    auto qos = camera_qos();

    auto pub_color = node_pub->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/bridge/sensors/color/compressed", qos);
    auto pub_depth = node_pub->create_publisher<sensor_msgs::msg::Image>(
        "/bridge/sensors/depth/image_raw", qos);

    // UniquePtr callbacks allow zero-copy forwarding within the same process.
    auto sub_color = node_sub->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/onboard/sensors/camera/color/image_raw/compressed", qos,
        [&pub_color](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
            pub_color->publish(std::move(msg));
        });

    auto sub_depth = node_sub->create_subscription<sensor_msgs::msg::Image>(
        "/onboard/sensors/camera/aligned_depth_to_color/image_raw", qos,
        [&pub_depth](sensor_msgs::msg::Image::UniquePtr msg) {
            pub_depth->publish(std::move(msg));
        });

    RCLCPP_INFO(node_sub->get_logger(),
        "camera_relay ready (domain %zu → %zu)", kDomainOnboard, kDomainBridge);

    // node_pub is publish-only — no need to spin it.
    // Spin node_sub in this thread; check g_stop every 100 ms.
    rclcpp::ExecutorOptions exec_opts;
    exec_opts.context = ctx_onboard;
    rclcpp::executors::SingleThreadedExecutor exec(exec_opts);
    exec.add_node(node_sub);
    while (!g_stop.load()) {
        exec.spin_some(std::chrono::milliseconds(100));
    }

    exec.remove_node(node_sub);
    node_sub.reset();
    node_pub.reset();
    rclcpp::shutdown(ctx_onboard);
    rclcpp::shutdown(ctx_bridge);
    return 0;
}
