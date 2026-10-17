#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr float kRedGain = 0.92F;
constexpr float kGreenGain = 2.08F;
constexpr float kBlueGain = 0.91F;
constexpr float kContrastPivot = 116.0F;
constexpr float kContrast = 1.10F;
constexpr float kMagentaSuppression = 1.0F;
constexpr float kToneGamma = 0.55F;
constexpr float kColorMix = 0.85F;

float corrected_channel(float value, float gain)
{
  return std::clamp(
    (value * gain - kContrastPivot) * kContrast + kContrastPivot,
    0.0F, 255.0F);
}

float tone_channel(float corrected)
{
  return 255.0F * std::pow(corrected / 255.0F, kToneGamma);
}

std::uint8_t corrected_byte(float value)
{
  return static_cast<std::uint8_t>(std::clamp(value, 0.0F, 255.0F) + 0.5F);
}

std::string build_pipeline(
  int sensor_id, int source_width, int source_height, double source_fps,
  int output_width, int output_height, double exposure_compensation)
{
  const int fps_value = std::max(1, static_cast<int>(source_fps + 0.5));
  return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
    " wbmode=1 awblock=false exposurecompensation=" +
    std::to_string(exposure_compensation) +
    " ! video/x-raw(memory:NVMM),width=" + std::to_string(source_width) +
    ",height=" + std::to_string(source_height) + ",framerate=" +
    std::to_string(fps_value) + "/1,format=NV12 ! "
    "nvvidconv interpolation-method=2 ! video/x-raw,width=" +
    std::to_string(output_width) + ",height=" + std::to_string(output_height) +
    ",format=BGRx ! appsink name=argus_sink drop=true max-buffers=1 sync=false";
}

class ArgusCameraNode final : public rclcpp::Node {
public:
  ArgusCameraNode()
  : Node("argus_camera_node")
  {
    declare_parameter<int>("sensor_id", 0);
    declare_parameter<std::string>("image_topic", "/stereo/front/image_raw");
    declare_parameter<std::string>("frame_id", "stereo_front_camera");
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<double>("fps", 30.0);
    declare_parameter<int>("source_width", 0);
    declare_parameter<int>("source_height", 0);
    declare_parameter<double>("source_fps", 0.0);
    declare_parameter<bool>("stereo_color_correction", true);
    declare_parameter<double>("stereo_exposure_compensation", 0.0);

    sensor_id_ = get_parameter("sensor_id").as_int();
    image_topic_ = get_parameter("image_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_double();
    source_width_ = get_parameter("source_width").as_int();
    source_height_ = get_parameter("source_height").as_int();
    source_fps_ = get_parameter("source_fps").as_double();
    color_correction_enabled_ = get_parameter("stereo_color_correction").as_bool();
    exposure_compensation_ = std::clamp(
      get_parameter("stereo_exposure_compensation").as_double(), -2.0, 2.0);

    if (source_width_ <= 0) {
      source_width_ = width_ >= 3840 && height_ >= 2160 ? 3840 : 1920;
    }
    if (source_height_ <= 0) {
      source_height_ = height_ >= 2160 && width_ >= 3840 ? 2160 : 1080;
    }
    if (source_fps_ <= 0.0) {
      source_fps_ = fps_;
    }

    const std::string pipeline_description = build_pipeline(
      sensor_id_, source_width_, source_height_, source_fps_, width_, height_,
      exposure_compensation_);
    RCLCPP_INFO(get_logger(), "opening Argus pipeline: %s", pipeline_description.c_str());

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

    sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "argus_sink"));
    if (sink_ == nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("failed to find GStreamer appsink");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS());
    const GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("failed to set Argus pipeline to PLAYING");
    }

    RCLCPP_INFO(
      get_logger(),
      "argus_camera_node ready sensor=%d topic=%s frame_id=%s source=%dx%d@%.1f "
      "output=%dx%d@%.1f encoding=bgr8 color_correction=%s",
      sensor_id_, image_topic_.c_str(), frame_id_.c_str(), source_width_, source_height_, source_fps_,
      width_, height_, fps_, color_correction_enabled_ ? "true" : "false");
    worker_ = std::thread(&ArgusCameraNode::capture_loop, this);
  }

  ~ArgusCameraNode() override
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
        for (int column = 0; column < frame_width; ++column) {
          const auto * source_pixel = source_row + column * 4;
          auto * target_pixel = target_row + column * 3;
          if (!color_correction_enabled_) {
            target_pixel[0] = source_pixel[0];
            target_pixel[1] = source_pixel[1];
            target_pixel[2] = source_pixel[2];
            continue;
          }

          float blue = corrected_channel(source_pixel[0], kBlueGain);
          float green = corrected_channel(source_pixel[1], kGreenGain);
          float red = corrected_channel(source_pixel[2], kRedGain);
          const float magenta = kMagentaSuppression * std::max(
            0.0F, std::min(red, blue) - green);
          red = tone_channel(std::clamp(red - 0.52F * magenta, 0.0F, 255.0F));
          green = tone_channel(std::clamp(green + 0.62F * magenta, 0.0F, 255.0F));
          blue = tone_channel(std::clamp(blue - 0.20F * magenta, 0.0F, 255.0F));
          const float gray = 0.299F * red + 0.587F * green + 0.114F * blue;
          const float gray_mix = 1.0F - kColorMix;
          target_pixel[0] = corrected_byte(kColorMix * blue + gray_mix * gray);
          target_pixel[1] = corrected_byte(kColorMix * green + gray_mix * gray);
          target_pixel[2] = corrected_byte(kColorMix * red + gray_mix * gray);
        }
      }
      gst_video_frame_unmap(&video_frame);
      gst_sample_unref(sample);

      publisher_->publish(message);
      ++publishes;
      const auto now_time = std::chrono::steady_clock::now();
      if (now_time - last_stats >= std::chrono::seconds(2)) {
        RCLCPP_INFO(
          get_logger(), "camera stats sensor=%d reads=%lu publishes=%lu",
          sensor_id_, static_cast<unsigned long>(reads), static_cast<unsigned long>(publishes));
        last_stats = now_time;
      }
    }
  }

  int sensor_id_{0};
  int width_{640};
  int height_{480};
  double fps_{30.0};
  int source_width_{1920};
  int source_height_{1080};
  double source_fps_{30.0};
  double exposure_compensation_{0.0};
  bool color_correction_enabled_{true};
  std::string image_topic_;
  std::string frame_id_;
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
    auto node = std::make_shared<ArgusCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("argus_camera_node"), "camera startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}
