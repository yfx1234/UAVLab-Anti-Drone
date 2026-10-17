#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

class SearchExposureGuard
{
public:
  ~SearchExposureGuard()
  {
    restore();
  }

  void configure(
    const std::string & device_path, int exposure, int gain, int brightness,
    int backlight_compensation)
  {
    device_path_ = device_path;
    active_ = true;
    try {
      run_control("auto_exposure=1");
      run_control("exposure_dynamic_framerate=0");
      run_control("exposure_time_absolute=" + std::to_string(exposure));
      run_control("gain=" + std::to_string(gain));
      run_control("brightness=" + std::to_string(brightness));
      run_control("backlight_compensation=" + std::to_string(backlight_compensation));
    } catch (...) {
      restore();
      throw;
    }
  }

  bool restore() noexcept
  {
    if (!active_) {
      return true;
    }
    active_ = false;
    const bool auto_ok = run_control_noexcept("auto_exposure=3");
    const bool priority_ok = run_control_noexcept("exposure_dynamic_framerate=1");
    return auto_ok && priority_ok;
  }

private:
  void run_control(const std::string & assignment) const
  {
    if (!run_control_noexcept(assignment)) {
      throw std::runtime_error(
              "v4l2-ctl failed for search camera control " + assignment);
    }
  }

  bool run_control_noexcept(const std::string & assignment) const noexcept
  {
    const std::string option = "--set-ctrl=" + assignment;
    const pid_t child = ::fork();
    if (child < 0) {
      return false;
    }
    if (child == 0) {
      ::execl(
        "/usr/bin/v4l2-ctl", "v4l2-ctl", "-d", device_path_.c_str(), option.c_str(),
        static_cast<char *>(nullptr));
      ::_exit(127);
    }
    int status = 0;
    return ::waitpid(child, &status, 0) == child && WIFEXITED(status) &&
           WEXITSTATUS(status) == 0;
  }

  std::string device_path_;
  bool active_{false};
};

std::string escape_device_path(const std::string &device_path)
{
  std::string escaped = device_path;
  std::string::size_type position = 0;
  while ((position = escaped.find('"', position)) != std::string::npos) {
    escaped.insert(position, "\\");
    position += 2;
  }
  return escaped;
}

std::string build_pipeline(
  const std::string &device_path,
  int source_width,
  int source_height,
  double source_fps,
  int output_width,
  int output_height)
{
  const int fps_value = std::max(1, static_cast<int>(source_fps + 0.5));
  return "v4l2src device=\"" + escape_device_path(device_path) +
    "\" ! image/jpeg,width=" + std::to_string(source_width) +
    ",height=" + std::to_string(source_height) + ",framerate=" +
    std::to_string(fps_value) +
    "/1 ! jpegparse ! jpegdec ! videoscale ! video/x-raw,width=" +
    std::to_string(output_width) + ",height=" + std::to_string(output_height) +
    ",format=I420 ! videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=usb_sink drop=true max-buffers=1 sync=false";
}

class UsbMjpegCameraNode final : public rclcpp::Node {
public:
  UsbMjpegCameraNode()
  : Node("usb_mjpeg_camera_node")
  {
    declare_parameter<std::string>("device_path", "/dev/video0");
    declare_parameter<std::string>("image_topic", "/search/image_raw");
    declare_parameter<std::string>("frame_id", "search_camera");
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<double>("fps", 25.0);
    declare_parameter<int>("source_width", 640);
    declare_parameter<int>("source_height", 480);
    declare_parameter<double>("source_fps", 25.0);
    declare_parameter<bool>("search_sun_filter", true);
    declare_parameter<int>("search_exposure_time", 5);
    declare_parameter<int>("search_gain", 0);
    declare_parameter<int>("search_brightness", -8);
    declare_parameter<int>("search_backlight_compensation", 0);

    device_path_ = get_parameter("device_path").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_double();
    source_width_ = get_parameter("source_width").as_int();
    source_height_ = get_parameter("source_height").as_int();
    source_fps_ = get_parameter("source_fps").as_double();
    search_sun_filter_ = get_parameter("search_sun_filter").as_bool();
    search_exposure_time_ = get_parameter("search_exposure_time").as_int();
    search_gain_ = get_parameter("search_gain").as_int();
    search_brightness_ = get_parameter("search_brightness").as_int();
    search_backlight_compensation_ =
      get_parameter("search_backlight_compensation").as_int();
    if (source_width_ <= 0) {
      source_width_ = width_;
    }
    if (source_height_ <= 0) {
      source_height_ = height_;
    }
    if (source_fps_ <= 0.0) {
      source_fps_ = fps_;
    }

    if (search_sun_filter_) {
      exposure_guard_.configure(
        device_path_, search_exposure_time_, search_gain_, search_brightness_,
        search_backlight_compensation_);
      RCLCPP_INFO(
        get_logger(),
        "search electronic ND enabled exposure=%d gain=%d brightness=%d backlight=%d",
        search_exposure_time_, search_gain_, search_brightness_, search_backlight_compensation_);
    }

    const std::string pipeline_description = build_pipeline(
      device_path_, source_width_, source_height_, source_fps_, width_, height_);
    RCLCPP_INFO(get_logger(), "opening USB MJPEG pipeline: %s", pipeline_description.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer parse error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error(message);
    }
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error(message);
    }

    sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "usb_sink"));
    if (sink_ == nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("failed to find USB GStreamer appsink");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS());
    const GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("failed to set USB GStreamer pipeline to PLAYING");
    }

    RCLCPP_INFO(
      get_logger(),
      "usb_mjpeg_camera_node ready device=%s topic=%s source=%dx%d@%.1f output=%dx%d@%.1f encoding=bgr8",
      device_path_.c_str(), image_topic_.c_str(), source_width_, source_height_, source_fps_,
      width_, height_, fps_);
    worker_ = std::thread(&UsbMjpegCameraNode::capture_loop, this);
  }

  ~UsbMjpegCameraNode() override
  {
    stopping_.store(true);
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    if (sink_ != nullptr) {
      gst_object_unref(sink_);
      sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    if (search_sun_filter_) {
      const bool restored = exposure_guard_.restore();
      RCLCPP_INFO(
        get_logger(), "search camera automatic exposure restore=%s",
        restored ? "success" : "failed");
    }
  }

private:
  void capture_loop()
  {
    auto last_stats = std::chrono::steady_clock::now();
    std::uint64_t reads = 0;
    std::uint64_t publishes = 0;
    while (!stopping_.load() && rclcpp::ok()) {
      GstSample *sample = gst_app_sink_pull_sample(sink_);
      if (sample == nullptr) {
        break;
      }
      ++reads;

      GstBuffer *buffer = gst_sample_get_buffer(sample);
      GstCaps *caps = gst_sample_get_caps(sample);
      GstVideoInfo video_info;
      GstVideoFrame video_frame;
      if (buffer == nullptr || caps == nullptr ||
        !gst_video_info_from_caps(&video_info, caps) ||
        !gst_video_frame_map(&video_frame, &video_info, buffer, GST_MAP_READ))
      {
        gst_sample_unref(sample);
        continue;
      }

      const int frame_width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&video_info));
      const int frame_height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&video_info));
      const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&video_frame, 0);
      const auto *source = static_cast<const std::uint8_t *>(
        GST_VIDEO_FRAME_PLANE_DATA(&video_frame, 0));
      auto message = sensor_msgs::msg::Image();
      const auto stamp = get_clock()->now();
      const auto stamp_nanoseconds = stamp.nanoseconds();
      message.header.stamp.sec = static_cast<int32_t>(stamp_nanoseconds / 1000000000LL);
      message.header.stamp.nanosec = static_cast<uint32_t>(stamp_nanoseconds % 1000000000LL);
      message.header.frame_id = frame_id_;
      message.height = frame_height;
      message.width = frame_width;
      message.encoding = "bgr8";
      message.is_bigendian = false;
      message.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_width * 3);
      message.data.resize(static_cast<size_t>(message.step) * frame_height);
      for (int row = 0; row < frame_height; ++row) {
        const auto *source_row = source + static_cast<size_t>(row) * stride;
        auto *target_row = message.data.data() + static_cast<size_t>(row) * message.step;
        std::memcpy(target_row, source_row, static_cast<size_t>(message.step));
      }
      gst_video_frame_unmap(&video_frame);
      gst_sample_unref(sample);

      publisher_->publish(message);
      ++publishes;
      const auto now = std::chrono::steady_clock::now();
      if (now - last_stats >= std::chrono::seconds(2)) {
        RCLCPP_INFO(
          get_logger(), "camera stats reads=%lu publishes=%lu",
          static_cast<unsigned long>(reads), static_cast<unsigned long>(publishes));
        last_stats = now;
      }
    }
  }

  std::string device_path_;
  std::string image_topic_;
  std::string frame_id_;
  int width_{640};
  int height_{480};
  double fps_{25.0};
  int source_width_{640};
  int source_height_{480};
  double source_fps_{25.0};
  bool search_sun_filter_{true};
  int search_exposure_time_{5};
  int search_gain_{0};
  int search_brightness_{-8};
  int search_backlight_compensation_{0};
  SearchExposureGuard exposure_guard_;
  GstElement *pipeline_{nullptr};
  GstAppSink *sink_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
};

}  // namespace

int main(int argc, char **argv)
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UsbMjpegCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("usb_mjpeg_camera_node"), "camera startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}
