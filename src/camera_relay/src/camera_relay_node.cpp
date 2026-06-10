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
 * Both callbacks return immediately (move into a KEEP_LAST=1 slot) so the
 * executor is never stalled by a slow domain-1 network write.  Dedicated
 * threads handle the publish independently.
 *
 * Depth uses RVL (Run-Length Variable-length, Wilson 2017): lossless,
 * 10-30× faster than PNG on ARM, no extra dependencies.
 * Wire format: "16UC1; compressedDepth rvl" (image_transport compatible).
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

    // --- color async publish thread ---
    std::mutex              color_mtx;
    std::condition_variable color_cv;
    sensor_msgs::msg::CompressedImage::UniquePtr pending_color;

    std::thread color_thread([&]() {
        while (!g_stop.load()) {
            sensor_msgs::msg::CompressedImage::UniquePtr msg;
            {
                std::unique_lock<std::mutex> lk(color_mtx);
                color_cv.wait_for(lk, std::chrono::milliseconds(200),
                    [&]{ return pending_color != nullptr || g_stop.load(); });
                msg = std::move(pending_color);
            }
            if (!msg) { continue; }
            pub_color->publish(std::move(msg));
        }
    });

    // --- depth async publish thread (RVL encode + publish) ---
    std::mutex              depth_mtx;
    std::condition_variable depth_cv;
    sensor_msgs::msg::Image::UniquePtr pending_depth;

    std::thread depth_thread([&]() {
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

            auto t0 = std::chrono::steady_clock::now();

            const int num_pixels = static_cast<int>(msg->width * msg->height);
            rvl_buf.resize(msg->data.size());

            int compressed_bytes = rvl::compress(
                reinterpret_cast<const uint16_t *>(msg->data.data()),
                rvl_buf.data(),
                num_pixels);

            auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
            out->header = std::move(msg->header);
            out->format = "16UC1; compressedDepth rvl";
            out->data.assign(rvl_buf.begin(), rvl_buf.begin() + compressed_bytes);
            pub_depth->publish(std::move(out));

            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            if (ms > 20) {
                RCLCPP_WARN(node_pub->get_logger(),
                    "[RELAY] depth compress+publish slow: %ldms (rvl=%dB)", ms, compressed_bytes);
            }
        }
    });

    // Both callbacks return immediately — executor is never stalled.
    auto sub_color = node_sub->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/onboard/sensors/camera/color/image_raw/compressed", qos,
        [&](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
            std::lock_guard<std::mutex> lk(color_mtx);
            pending_color = std::move(msg);
            color_cv.notify_one();
        });

    // Diagnostic: distinguish SDK frame drop vs DDS/ROS delivery lag.
    //   stamp_gap  = header.stamp interval → camera SDK dropped a frame if > 50ms
    //   wall_gap   = wall-clock callback interval → delivery lag if > stamp_gap
    auto last_depth_stamp_ns = std::make_shared<int64_t>(0);
    auto last_depth_cb       = std::make_shared<std::chrono::steady_clock::time_point>();
    auto sub_depth = node_sub->create_subscription<sensor_msgs::msg::Image>(
        "/onboard/sensors/camera/aligned_depth_to_color/image_raw", qos,
        [&, last_depth_stamp_ns, last_depth_cb](sensor_msgs::msg::Image::UniquePtr msg) {
            auto now      = std::chrono::steady_clock::now();
            int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

            if (*last_depth_stamp_ns > 0) {
                double stamp_gap_ms = (stamp - *last_depth_stamp_ns) / 1e6;
                double wall_gap_ms  = std::chrono::duration<double, std::milli>(
                    now - *last_depth_cb).count();

                if (stamp_gap_ms > 50.0) {
                    RCLCPP_WARN(node_sub->get_logger(),
                        "[SDK-DROP] depth stamp gap: %.0fms  wall: %.0fms  "
                        "(camera dropped ~%.0f frames)",
                        stamp_gap_ms, wall_gap_ms, stamp_gap_ms / 33.3 - 1.0);
                } else if (wall_gap_ms > 50.0) {
                    RCLCPP_WARN(node_sub->get_logger(),
                        "[DDS-LAG]  depth stamp gap: %.0fms  wall: %.0fms  "
                        "(delivery delayed, frames queued in DDS)",
                        stamp_gap_ms, wall_gap_ms);
                }
            }
            *last_depth_stamp_ns = stamp;
            *last_depth_cb       = now;

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

    color_thread.join();
    depth_thread.join();
    exec.remove_node(node_sub);
    node_sub.reset();
    node_pub.reset();
    rclcpp::shutdown(ctx_onboard);
    rclcpp::shutdown(ctx_bridge);
    return 0;
}
