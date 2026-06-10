/**
 * camera_relay — C++ domain bridge for camera frames.
 *
 * Relays camera topics from domain 0 (onboard) to domain 1 (bridge).
 *
 * Subscriptions  (domain 0, BEST_EFFORT depth=1):
 *   /onboard/sensors/camera/color/image_raw/compressed        (CompressedImage)
 *   /onboard/sensors/camera/aligned_depth_to_color/image_raw  (Image, 16UC1)
 *
 * Publications   (domain 1, BEST_EFFORT depth=1):
 *   /bridge/sensors/color/compressed       (CompressedImage, JPEG)
 *   /bridge/sensors/depth/compressedDepth  (CompressedImage, PNG 16-bit)
 *
 * Depth is PNG-encoded in a dedicated thread (PNG level 1, ~5-15 ms on ARM)
 * so the executor never stalls on the 1.8 MB raw frame.  The async slot keeps
 * only the latest frame (KEEP_LAST=1 semantics), so compression back-pressure
 * drops stale frames rather than queuing them.
 *
 * image_transport compressedDepth wire format for 16UC1:
 *   format = "16UC1; compressedDepth png"
 *   data   = raw PNG bytes (no extra header needed for integer depth)
 */
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/imgcodecs.hpp>
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
    auto pub_depth = node_pub->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/bridge/sensors/depth/compressedDepth", qos);

    // Depth: raw Image arrives at 30 Hz; callback stores latest frame and
    // returns immediately.  A dedicated thread PNG-encodes (level 1) and
    // publishes — slow UDP writes on domain 1 never back up the executor.
    std::mutex              depth_mtx;
    std::condition_variable depth_cv;
    sensor_msgs::msg::Image::UniquePtr pending_depth;

    std::thread depth_thread([&]() {
        while (!g_stop.load()) {
            sensor_msgs::msg::Image::UniquePtr msg;
            {
                std::unique_lock<std::mutex> lk(depth_mtx);
                depth_cv.wait_for(lk, std::chrono::milliseconds(200),
                    [&]{ return pending_depth != nullptr || g_stop.load(); });
                msg = std::move(pending_depth);
            }
            if (!msg) { continue; }

            cv::Mat mat(static_cast<int>(msg->height),
                        static_cast<int>(msg->width),
                        CV_16UC1,
                        const_cast<uint8_t *>(msg->data.data()));
            std::vector<uint8_t> png_buf;
            cv::imencode(".png", mat, png_buf,
                         {cv::IMWRITE_PNG_COMPRESSION, 1});

            auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
            out->header = std::move(msg->header);
            out->format = "16UC1; compressedDepth png";
            out->data   = std::move(png_buf);
            pub_depth->publish(std::move(out));
        }
    });

    // Color: already compressed by the camera driver — zero-copy forward.
    auto sub_color = node_sub->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/onboard/sensors/camera/color/image_raw/compressed", qos,
        [&pub_color](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
            pub_color->publish(std::move(msg));
        });

    auto sub_depth = node_sub->create_subscription<sensor_msgs::msg::Image>(
        "/onboard/sensors/camera/aligned_depth_to_color/image_raw", qos,
        [&](sensor_msgs::msg::Image::UniquePtr msg) {
            std::lock_guard<std::mutex> lk(depth_mtx);
            pending_depth = std::move(msg);
            depth_cv.notify_one();
        });

    RCLCPP_INFO(node_sub->get_logger(),
        "camera_relay ready (domain %zu → %zu)", kDomainOnboard, kDomainBridge);

    rclcpp::ExecutorOptions exec_opts;
    exec_opts.context = ctx_onboard;
    rclcpp::executors::SingleThreadedExecutor exec(exec_opts);
    exec.add_node(node_sub);
    while (!g_stop.load()) {
        exec.spin_some(std::chrono::milliseconds(100));
    }

    depth_thread.join();
    exec.remove_node(node_sub);
    node_sub.reset();
    node_pub.reset();
    rclcpp::shutdown(ctx_onboard);
    rclcpp::shutdown(ctx_bridge);
    return 0;
}
