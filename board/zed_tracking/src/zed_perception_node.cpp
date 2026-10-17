// zed_perception_node:ZED X Mini 单进程感知节点。
//
// 线程模型:
//   采集线程 —— 30Hz grab(SVO 录像由 SDK 随 grab 自动写入),每 N 帧把
//              左目 BGRA + 深度 + 置信度拷贝进有界队列(满则丢最旧)。
//   推理线程 —— 取最新帧,letterbox 到模型输入,同步推理,解析检测,
//              深度中值解算三维,坐标链换算,发布轻量 JSON。
//   主线程   —— ROS spin 与 1Hz 状态发布。
//
// 输出话题(均在节点命名空间下):
//   detections_json —— 兼容旧 /search/detections_json 合同,多带字段无害;
//   target_json     —— 单目标三维观测(深度/伪测距双模式);
//   status          —— 运行状态 JSON。

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "zed_tracking/depth_stats.hpp"
#include "zed_tracking/geometry.hpp"
#include "zed_tracking/yolo_decode.hpp"
#include "trt_runner.hpp"
#include "zed_source.hpp"

namespace zed_tracking
{

namespace
{

struct FramePacket
{
  std::int64_t stamp_ns{0};
  std::uint32_t sequence{0};
  int width{0};
  int height{0};
  bool has_depth{false};
  std::vector<unsigned char> bgra;
  std::vector<float> depth;
  std::vector<float> confidence;
};

using Clock = std::chrono::steady_clock;

class RollingRate
{
public:
  void tick()
  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (valid_) {
      const double seconds =
        std::chrono::duration<double>(now - last_).count();
      if (seconds > 0.0 && seconds < 5.0) {
        instantaneous_ = 1.0 / seconds;
      }
    }
    last_ = now;
    valid_ = true;
  }
  double rate() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return instantaneous_;
  }

private:
  mutable std::mutex mutex_;
  Clock::time_point last_{};
  bool valid_{false};
  double instantaneous_{0.0};
};

std::string format_timestamp_path(const char * prefix, const char * suffix)
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char buffer[128];
  std::snprintf(
    buffer, sizeof(buffer), "%s%04d%02d%02d_%02d%02d%02d%s", prefix,
    local.tm_year + 1900, local.tm_mon + 1, local.tm_mday, local.tm_hour,
    local.tm_min, local.tm_sec, suffix);
  return buffer;
}

}  // namespace

class ZedPerceptionNode : public rclcpp::Node
{
public:
  ZedPerceptionNode()
  : Node("zed_perception_node")
  {
    load_parameters();
    detection_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/detections_json", 10);
    target_publisher_ =
      create_publisher<std_msgs::msg::String>("~/target_json", 10);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>("~/status", 10);
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_status(); });

    runner_ = std::make_unique<TrtRunner>();
    runner_->load(engine_path_);
    RCLCPP_INFO(
      get_logger(), "engine %s input %dx%d output dims %s", engine_path_.c_str(),
      runner_->input_width(), runner_->input_height(),
      shape_string(runner_->output_shape()).c_str());

    ZedAcquireConfig acquire;
    acquire.resolution = resolution_;
    acquire.fps = camera_fps_;
    acquire.depth_mode = depth_mode_;
    acquire.depth_min_m = depth_min_m_;
    acquire.depth_max_m = depth_max_m_;
    acquire.runtime_confidence = runtime_confidence_;
    acquire.input_svo = input_svo_;
    zed_->open(acquire);
    intrinsics_ = zed_->intrinsics();
    if (record_enabled_) {
      start_recording();
    }

    stop_.store(false);
    capture_thread_ = std::thread([this]() { capture_loop(); });
    infer_thread_ = std::thread([this]() { infer_loop(); });
    RCLCPP_INFO(
      get_logger(), "zed_perception ready: %s@%d depth=%s detect_every=%d",
      resolution_.c_str(), camera_fps_, depth_mode_.c_str(), detect_every_n_);
  }

  ~ZedPerceptionNode() override
  {
    stop_.store(true);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_cv_.notify_all();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (infer_thread_.joinable()) {
      infer_thread_.join();
    }
    try {
      zed_->close();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "close error: %s", error.what());
    }
  }

private:
  void load_parameters()
  {
    const auto string_parameter = [this](const char * name, const char * fallback) {
        return declare_parameter<std::string>(name, fallback);
      };
    const auto double_parameter = [this](const char * name, double fallback) {
        return declare_parameter<double>(name, fallback);
      };
    const auto int_parameter = [this](const char * name, int fallback) {
        return declare_parameter<int>(name, fallback);
      };

    resolution_ = string_parameter("resolution", "HD1080");
    camera_fps_ = int_parameter("camera_fps", 30);
    depth_mode_ = string_parameter("depth_mode", "NEURAL_LIGHT");
    depth_min_m_ = double_parameter("depth_min_m", 0.3);
    depth_max_m_ = double_parameter("depth_max_m", 12.0);
    runtime_confidence_ = int_parameter("runtime_confidence", 80);
    detect_every_n_ = std::max<int>(1, int_parameter("detect_every_n", 2));
    input_svo_ = string_parameter("input_svo", "");
    // 回放模式:SVO 内是录制时刻的历史时间戳,重打为当前时钟,
    // 下游(姿态配对/滤波)才能当作实时流处理。
    restamp_output_ = declare_parameter<bool>("restamp_output", false);

    engine_path_ = string_parameter("engine_path", "");
    if (engine_path_.empty()) {
      throw std::runtime_error("engine_path parameter is required");
    }
    score_threshold_ = double_parameter("score_threshold", 0.55);
    iou_threshold_ = double_parameter("iou_threshold", 0.45);
    class_count_ = std::max<int>(1, int_parameter("class_count", 1));

    depth_box_shrink_ = double_parameter("depth_box_shrink", 0.6);
    depth_min_valid_ = std::max<int>(1, int_parameter("depth_min_valid_pixels", 12));
    depth_confidence_max_ = double_parameter("depth_confidence_max", 95.0);
    depth_usable_max_m_ = double_parameter("depth_usable_max_m", 10.5);

    target_size_m_ = double_parameter("target_size_m", 0.30);
    reference_range_m_ = double_parameter("reference_range_m", 10.0);
    reference_bbox_px_ = double_parameter("reference_bbox_px", 33.0);
    reference_image_width_ = double_parameter("reference_image_width", 1920.0);

    const auto quaternion = declare_parameter<std::vector<double>>(
      "q_body_from_camera_wxyz", {1.0, 0.0, 0.0, 0.0});
    if (quaternion.size() != 4) {
      throw std::runtime_error("q_body_from_camera_wxyz must have 4 elements");
    }
    extrinsic_quaternion_ =
      Quat{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
    const auto translation = declare_parameter<std::vector<double>>(
      "camera_translation_body_m", {0.10, 0.0, 0.12});
    if (translation.size() != 3) {
      throw std::runtime_error("camera_translation_body_m must have 3 elements");
    }
    extrinsic_translation_ =
      Vec3{translation[0], translation[1], translation[2]};

    record_enabled_ = declare_parameter<bool>("record_enabled", true);
    svo_directory_ = string_parameter(
      "svo_directory", "/home/uavlab/Jetson_ROS2_V0/flight_zed_records");
    svo_compression_ = string_parameter("svo_compression", "H264");
    snapshot_enabled_ = declare_parameter<bool>("snapshot_enabled", true);
    snapshot_directory_ = string_parameter("snapshot_directory", "/tmp/zed_tracking");
    snapshot_period_s_ = double_parameter("snapshot_period_s", 1.0);
  }

  void start_recording()
  {
    std::string command = "mkdir -p " + svo_directory_;
    if (std::system(command.c_str()) != 0) {
      RCLCPP_WARN(get_logger(), "mkdir failed for %s", svo_directory_.c_str());
    }
    svo_path_ = format_timestamp_path(
      (svo_directory_ + "/zed_").c_str(), ".svo");
    zed_->start_recording(svo_path_, svo_compression_);
    RCLCPP_INFO(get_logger(), "SVO recording to %s", svo_path_.c_str());
  }

  static std::string shape_string(const std::vector<std::int64_t> & dims)
  {
    std::string text = "[";
    for (std::size_t index = 0; index < dims.size(); ++index) {
      if (index) {
        text += ",";
      }
      text += std::to_string(dims[index]);
    }
    return text + "]";
  }

  void capture_loop()
  {
    std::uint32_t sequence = 0;
    int consecutive_failures = 0;
    while (!stop_.load() && rclcpp::ok()) {
      if (!zed_->grab()) {
        if (++consecutive_failures > 60) {
          RCLCPP_ERROR(get_logger(), "ZED grab failed 60 times, stopping capture");
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      consecutive_failures = 0;
      ++sequence;
      capture_rate_.tick();
      if (static_cast<int>(sequence % detect_every_n_) != 0) {
        continue;
      }
      auto packet = std::make_shared<FramePacket>();
      packet->sequence = sequence;
      packet->stamp_ns = zed_->image_timestamp_ns();
      try {
        const sl::Mat & left = zed_->retrieve_left();
        packet->width = static_cast<int>(left.getWidth());
        packet->height = static_cast<int>(left.getHeight());
        const std::size_t row_bytes =
          static_cast<std::size_t>(packet->width) * 4u;
        packet->bgra.resize(row_bytes * packet->height);
        const auto * source = reinterpret_cast<const unsigned char *>(
          left.getPtr<sl::uchar1>());
        const std::size_t stride = left.getStepBytes();
        for (int row = 0; row < packet->height; ++row) {
          std::memcpy(
            &packet->bgra[static_cast<std::size_t>(row) * row_bytes],
            source + row * stride, row_bytes);
        }
        if (depth_mode_ != "NONE") {
          const sl::Mat & depth = zed_->retrieve_depth();
          const sl::Mat & confidence = zed_->retrieve_confidence();
          const std::size_t pixels =
            static_cast<std::size_t>(packet->width) * packet->height;
          packet->depth.resize(pixels);
          packet->confidence.resize(pixels);
          const float * depth_source = depth.getPtr<float>();
          const float * confidence_source = confidence.getPtr<float>();
          const std::size_t depth_stride = depth.getStepBytes() / sizeof(float);
          const std::size_t confidence_stride =
            confidence.getStepBytes() / sizeof(float);
          for (int row = 0; row < packet->height; ++row) {
            std::memcpy(
              &packet->depth[static_cast<std::size_t>(row) * packet->width],
              depth_source + row * depth_stride,
              static_cast<std::size_t>(packet->width) * sizeof(float));
            std::memcpy(
              &packet->confidence[static_cast<std::size_t>(row) * packet->width],
              confidence_source + row * confidence_stride,
              static_cast<std::size_t>(packet->width) * sizeof(float));
          }
          packet->has_depth = true;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "retrieve error: %s", error.what());
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (queue_.size() >= 2) {
          queue_.pop();
        }
        queue_.push(std::move(packet));
        queue_size_.store(static_cast<int>(queue_.size()));
        queue_cv_.notify_one();
      }
    }
  }

  void infer_loop()
  {
    const int input_width = runner_->input_width();
    const int input_height = runner_->input_height();
    std::vector<float> host_input(
      static_cast<std::size_t>(3) * input_width * input_height);
    auto last_snapshot = Clock::now() - std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(snapshot_period_s_));
    while (!stop_.load()) {
      std::shared_ptr<FramePacket> packet;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(
          lock, [this]() { return stop_.load() || !queue_.empty(); });
        if (stop_.load() || queue_.empty()) {
          break;
        }
        packet = queue_.front();
        queue_.pop();
        queue_size_.store(static_cast<int>(queue_.size()));
      }
      const auto started = Clock::now();

      cv::Mat source(
        packet->height, packet->width, CV_8UC4, packet->bgra.data());
      const Letterbox box = make_letterbox(
        packet->width, packet->height, input_width, input_height);
      const int scaled_width = static_cast<int>(
        std::lround(packet->width * box.scale));
      const int scaled_height = static_cast<int>(
        std::lround(packet->height * box.scale));
      cv::Mat resized;
      cv::resize(
        source, resized, cv::Size(scaled_width, scaled_height), 0, 0,
        cv::INTER_LINEAR);
      const int pad_top = box.pad_y;
      const int pad_left = box.pad_x;
      const int pad_bottom = input_height - scaled_height - pad_top;
      const int pad_right = input_width - scaled_width - pad_left;
      cv::Mat letterboxed;
      cv::copyMakeBorder(
        resized, letterboxed, pad_top, pad_bottom, pad_left, pad_right,
        cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114, 255));

      const std::size_t plane =
        static_cast<std::size_t>(input_width) * input_height;
      for (int row = 0; row < input_height; ++row) {
        const auto * line =
          letterboxed.ptr<cv::Vec4b>(row);
        float * plane_r = host_input.data() + row * input_width;
        float * plane_g = host_input.data() + plane + row * input_width;
        float * plane_b = host_input.data() + 2 * plane + row * input_width;
        for (int col = 0; col < input_width; ++col) {
          plane_r[col] = line[col][2] * (1.0F / 255.0F);
          plane_g[col] = line[col][1] * (1.0F / 255.0F);
          plane_b[col] = line[col][0] * (1.0F / 255.0F);
        }
      }

      const std::vector<float> & output = runner_->infer(host_input.data());
      std::vector<Detection> detections = decode_detections(
        output.data(), runner_->output_shape(), class_count_,
        score_threshold_, iou_threshold_);
      for (auto & detection : detections) {
        double x1 = detection.x1, y1 = detection.y1;
        double x2 = detection.x2, y2 = detection.y2;
        map_box_back(box, &x1, &y1, &x2, &y2);
        detection.x1 = static_cast<float>(x1);
        detection.y1 = static_cast<float>(y1);
        detection.x2 = static_cast<float>(x2);
        detection.y2 = static_cast<float>(y2);
      }
      std::sort(
        detections.begin(), detections.end(),
        [](const Detection & a, const Detection & b) {return a.score > b.score;});

      const double stamp_s = restamp_output_
        ? static_cast<double>(this->get_clock()->now().nanoseconds()) / 1.0e9
        : static_cast<double>(packet->stamp_ns) / 1.0e9;
      publish_detections(stamp_s, packet->sequence, packet->width,
        packet->height, detections);
      if (!detections.empty()) {
        resolve_target(
          *packet, detections.front(), stamp_s, intrinsics_,
          box);
      } else {
        publish_target_none(stamp_s, packet->sequence);
      }

      infer_rate_.tick();
      last_infer_ms_.store(
        std::chrono::duration<double, std::milli>(Clock::now() - started)
        .count());
      const auto now = Clock::now();
      if (snapshot_enabled_ &&
        std::chrono::duration<double>(now - last_snapshot).count() >=
        snapshot_period_s_)
      {
        last_snapshot = now;
        write_snapshot(packet, detections, source);
      }
    }
  }

  void resolve_target(
    const FramePacket & packet, const Detection & detection,
    double stamp_s, const CameraIntrinsics & intrinsics, const Letterbox & box)
  {
    (void)box;
    const double u = 0.5 * (detection.x1 + detection.x2);
    const double v = 0.5 * (detection.y1 + detection.y2);
    const double bbox_long =
      std::max(detection.x2 - detection.x1, detection.y2 - detection.y1);
    const double focal = 0.5 * (intrinsics.fx + intrinsics.fy);

    DepthSample sample;
    std::string mode = "pseudo";
    double range_m = 0.0;
    if (packet.has_depth) {
      sample = median_depth_in_box(
        packet.depth.data(), packet.width, packet.confidence.data(),
        packet.width, packet.width, packet.height,
        static_cast<int>(std::floor(detection.x1)),
        static_cast<int>(std::floor(detection.y1)),
        static_cast<int>(std::ceil(detection.x2)),
        static_cast<int>(std::ceil(detection.y2)),
        depth_box_shrink_, depth_min_valid_, depth_min_m_,
        depth_usable_max_m_, depth_confidence_max_);
      if (sample.ok) {
        mode = "depth";
        range_m = sample.median_m;
      }
    }
    const double range_physical =
      pseudo_range_physical(target_size_m_, focal, bbox_long);
    const double range_reference = pseudo_range_reference(
      bbox_long, packet.width, reference_range_m_, reference_bbox_px_,
      reference_image_width_);
    if (mode != "depth") {
      range_m = range_physical;
    }
    const Vec3 camera_point =
      pixel_to_camera(u, v, range_m, intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy);
    const Vec3 body_point =
      camera_to_body(extrinsic_quaternion_, extrinsic_translation_, camera_point);

    char text[1024];
    std::snprintf(
      text, sizeof(text),
      "{\"stamp_s\":%.6f,\"sequence\":%u,\"mode\":\"%s\",\"confidence\":%.3f,"
      "\"u_px\":%.1f,\"v_px\":%.1f,\"bbox_long_px\":%.1f,"
      "\"depth_m\":%s,\"depth_valid_pixels\":%d,\"depth_p25_m\":%.2f,"
      "\"depth_p75_m\":%.2f,"
      "\"range_physical_m\":%.2f,\"range_reference_m\":%.2f,\"range_m\":%.2f,"
      "\"camera_xyz\":[%.3f,%.3f,%.3f],\"body_xyz\":[%.3f,%.3f,%.3f]}",
      stamp_s, packet.sequence, mode.c_str(), detection.score, u, v, bbox_long,
      sample.ok ? std::to_string(sample.median_m).c_str() : "null",
      sample.valid_pixels, sample.ok ? sample.p25_m : 0.0,
      sample.ok ? sample.p75_m : 0.0,
      range_physical, range_reference, range_m,
      camera_point.x, camera_point.y, camera_point.z,
      body_point.x, body_point.y, body_point.z);
    std_msgs::msg::String message;
    message.data = text;
    target_publisher_->publish(message);
    if (mode != last_mode_) {
      RCLCPP_INFO(
        get_logger(), "target mode -> %s (range %.2f m, bbox %.0f px)", mode.c_str(),
        range_m, bbox_long);
      last_mode_ = mode;
    }
  }

  void publish_target_none(double stamp_s, std::uint32_t sequence)
  {
    char text[160];
    std::snprintf(
      text, sizeof(text),
      "{\"stamp_s\":%.6f,\"sequence\":%u,\"mode\":\"none\"}", stamp_s, sequence);
    std_msgs::msg::String message;
    message.data = text;
    target_publisher_->publish(message);
    last_mode_ = "none";
  }

  void publish_detections(
    double stamp_s, std::uint32_t sequence, int width, int height,
    const std::vector<Detection> & detections)
  {
    std::string text;
    text.reserve(256 + detections.size() * 96);
    char header[256];
    std::snprintf(
      header, sizeof(header),
      "{\"header\":{\"stamp\":{\"sec\":%d,\"nanosec\":%u},"
      "\"frame_id\":\"zed_left\"},\"sequence\":%u,"
      "\"image_width\":%d,\"image_height\":%d,\"source\":\"zed_x_mini\","
      "\"detections\":[",
      static_cast<int>(stamp_s), static_cast<unsigned>(std::lround(
        (stamp_s - std::floor(stamp_s)) * 1.0e9)),
      sequence, width, height);
    text += header;
    for (std::size_t index = 0; index < detections.size(); ++index) {
      const auto & detection = detections[index];
      char item[128];
      std::snprintf(
        item, sizeof(item),
        "%s{\"confidence\":%.3f,\"bbox_xyxy\":[%.1f,%.1f,%.1f,%.1f],"
        "\"class_id\":%d}",
        index ? "," : "", detection.score, detection.x1, detection.y1,
        detection.x2, detection.y2, detection.class_id);
      text += item;
    }
    text += "]}";
    std_msgs::msg::String message;
    message.data = text;
    detection_publisher_->publish(message);
  }

  void write_snapshot(
    const std::shared_ptr<FramePacket> & packet,
    const std::vector<Detection> & detections, const cv::Mat & source)
  {
    try {
      cv::Mat bgr;
      cv::cvtColor(source, bgr, cv::COLOR_BGRA2BGR);
      for (const auto & detection : detections) {
        cv::rectangle(
          bgr,
          cv::Point(
            static_cast<int>(detection.x1), static_cast<int>(detection.y1)),
          cv::Point(
            static_cast<int>(detection.x2), static_cast<int>(detection.y2)),
          cv::Scalar(0, 255, 0), 2);
        char label[64];
        std::snprintf(label, sizeof(label), "%.2f", detection.score);
        cv::putText(
          bgr, label,
          cv::Point(
            static_cast<int>(detection.x1),
            std::max(14, static_cast<int>(detection.y1) - 6)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
      static std::mutex snapshot_mutex;
      std::lock_guard<std::mutex> lock(snapshot_mutex);
      std::string command = "mkdir -p " + snapshot_directory_;
      if (std::system(command.c_str()) == 0) {
        cv::imwrite(snapshot_directory_ + "/latest.jpg", bgr);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "snapshot error: %s", error.what());
    }
  }

  void publish_status()
  {
    char text[512];
    std::snprintf(
      text, sizeof(text),
      "{\"capture_fps\":%.1f,\"infer_fps\":%.1f,\"infer_ms\":%.1f,"
      "\"mode\":\"%s\",\"recording\":%s,\"svo\":\"%s\",\"engine\":\"%s\","
      "\"resolution\":\"%s\",\"depth_mode\":\"%s\",\"queue\":%d}",
      capture_rate_.rate(), infer_rate_.rate(), last_infer_ms_.load(),
      last_mode_.c_str(), zed_->recording() ? "true" : "false",
      svo_path_.c_str(), engine_path_.c_str(), resolution_.c_str(),
      depth_mode_.c_str(), queue_size_.load());
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }

  std::string resolution_{"HD1080"};
  int camera_fps_{30};
  std::string depth_mode_{"NEURAL_LIGHT"};
  std::string input_svo_;
  bool restamp_output_{false};
  double depth_min_m_{0.3};
  double depth_max_m_{12.0};
  int runtime_confidence_{80};
  int detect_every_n_{2};
  std::string engine_path_;
  double score_threshold_{0.55};
  double iou_threshold_{0.45};
  int class_count_{1};
  double depth_box_shrink_{0.6};
  int depth_min_valid_{12};
  double depth_confidence_max_{95.0};
  double depth_usable_max_m_{10.5};
  double target_size_m_{0.30};
  double reference_range_m_{10.0};
  double reference_bbox_px_{33.0};
  double reference_image_width_{1920.0};
  Quat extrinsic_quaternion_{};
  Vec3 extrinsic_translation_{};
  bool record_enabled_{true};
  std::string svo_directory_;
  std::string svo_compression_{"H264"};
  std::string svo_path_;
  bool snapshot_enabled_{true};
  std::string snapshot_directory_;
  double snapshot_period_s_{1.0};

  std::unique_ptr<ZedSource> zed_ = std::make_unique<ZedSource>();
  std::unique_ptr<TrtRunner> runner_;
  CameraIntrinsics intrinsics_{};
  std::string last_mode_{"none"};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::thread capture_thread_;
  std::thread infer_thread_;
  std::queue<std::shared_ptr<FramePacket>> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> stop_{false};
  std::atomic<int> queue_size_{0};
  RollingRate capture_rate_;
  RollingRate infer_rate_;
  std::atomic<double> last_infer_ms_{0.0};
};

}  // namespace zed_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<zed_tracking::ZedPerceptionNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "zed_perception_node fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
