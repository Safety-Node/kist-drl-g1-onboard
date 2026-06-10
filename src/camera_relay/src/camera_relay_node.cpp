/**
 * camera_relay — C++ domain bridge for camera frames.
 *
 * Subscriptions  (domain 0, BEST_EFFORT depth=1):
 *   /onboard/sensors/camera/color/image_raw/compressed        (CompressedImage)
 *   /onboard/sensors/camera/aligned_depth_to_color/image_raw  (Image, 16UC1)
 *
 * Publications   (domain 1, BEST_EFFORT depth=1):
 *   /bridge/sensors/color/compressed       (CompressedImage, JPEG)
 *   /bridge/sensors/depth/compressedDepth  (CompressedImage, RVL)
 *
 * Depth is RVL-encoded in a dedicated thread.  RVL (Run-Length Variable-
 * length, Wilson 2017) is lossless and 10-30× faster than PNG on ARM because
 * it only scans the pixel array once with no entropy coding.
 * Wire format matches image_transport compressedDepth:
 *   format = "16UC1; compressedDepth rvl"
 *   data   = raw RVL bytes (no extra header for integer depth)
 */
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// ---------------------------------------------------------------------------
// RVL encoder — Andrew Wilson, "Fast Lossless Depth Image Compression" (2017)
// Encodes 16-bit depth pixel-deltas with nibble-based variable-length coding.
// Zero runs and non-zero delta runs are encoded separately for speed.
// ---------------------------------------------------------------------------
namespace rvl {

static int compress(const uint16_t * input, uint8_t * output, int num_pixels)
{
    int * p       = reinterpret_cast<int *>(output);
    int   word    = 0;
    int   nibbles = 0;

    const auto encode = [&](int v) {
        do {
            int n = v & 0x7; v >>= 3;
            if (v) n |= 0x8;
            word = (word << 4) | n;
            if (++nibbles == 8) { *p++ = word; nibbles = 0; word = 0; }
        } while (v);
    };

    const uint16_t * end  = input + num_pixels;
    uint16_t         prev = 0;
    while (input != end) {
        int zeros = 0, nz = 0;
        for (; input != end && !*input; ++input, ++zeros);
        encode(zeros);
        for (const uint16_t * q = input; q != end && *q; ++q, ++nz);
        encode(nz);
        for (int i = 0; i < nz; ++i) {
            uint16_t cur = *input++;
            int d = static_cast<int>(cur) - static_cast<int>(prev);
            encode((d << 1) ^ (d >> 15));
            prev = cur;
        }
    }
    if (nibbles) { *p++ = word << ((8 - nibbles) * 4); }
    return static_cast<int>(reinterpret_cast<uint8_t *>(p) - output);
}

}  // namespace rvl

// ---------------------------------------------------------------------------

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

    // Depth: callback stores latest frame; dedicated thread RVL-encodes and
    // publishes.  KEEP_LAST=1 slot drops stale frames under back-pressure.
    std::mutex              depth_mtx;
    std::condition_variable depth_cv;
    sensor_msgs::msg::Image::UniquePtr pending_depth;

    std::thread depth_thread([&]() {
        // Pre-allocate worst-case RVL buffer (same size as raw frame).
        std::vector<uint8_t> rvl_buf;

        while (!g_stop.load()) {
            sensor_msgs::msg::Image::UniquePtr msg;
            {
                std::unique_lock<std::mutex> lk(depth_mtx);
                depth_cv.wait_for(lk, std::chrono::milliseconds(200),
                    [&]{ return pending_depth != nullptr || g_stop.load(); });
                msg = std::move(pending_depth);
            }
            if (!msg) { continue; }

            const int num_pixels = static_cast<int>(msg->width * msg->height);
            rvl_buf.resize(msg->data.size());  // upper bound

            int compressed_bytes = rvl::compress(
                reinterpret_cast<const uint16_t *>(msg->data.data()),
                rvl_buf.data(),
                num_pixels);

            auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
            out->header = std::move(msg->header);
            out->format = "16UC1; compressedDepth rvl";
            out->data.assign(rvl_buf.begin(), rvl_buf.begin() + compressed_bytes);
            pub_depth->publish(std::move(out));
        }
    });

    // Color: already JPEG-compressed by the driver — zero-copy forward.
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
