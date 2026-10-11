// ZED SDK 5.5 采集封装:打开、抓帧、左目/深度/置信度取图、内参、SVO 录像。
// 用法参考板上已验证程序 ~/zed_preview/zed_preview.cpp 与 ~/test_zed_depth.cpp。

#pragma once

#include <sl/Camera.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

namespace zed_tracking
{

struct ZedAcquireConfig
{
  std::string resolution{"HD1080"};      // HD2K/HD1200/HD1080/HD720/SVGA/VGA
  int fps{30};
  std::string depth_mode{"NEURAL_LIGHT"};  // NEURAL_LIGHT/NEURAL/PERFORMANCE/NONE
  double depth_min_m{0.3};
  double depth_max_m{12.0};
  int runtime_confidence{80};             // SDK 置信度阈值,越低越严格
  std::string input_svo;                  // 非空则回放 SVO 而非实时相机
  bool svo_real_time{true};               // 按录制节拍回放
};

inline sl::RESOLUTION parse_resolution(const std::string & value)
{
  if (value == "HD2K") {return sl::RESOLUTION::HD2K;}
  if (value == "HD1536") {return sl::RESOLUTION::HD1536;}
  if (value == "HD1200") {return sl::RESOLUTION::HD1200;}
  if (value == "HD1080") {return sl::RESOLUTION::HD1080;}
  if (value == "HD720") {return sl::RESOLUTION::HD720;}
  if (value == "SVGA") {return sl::RESOLUTION::SVGA;}
  if (value == "VGA") {return sl::RESOLUTION::VGA;}
  if (value == "XVGA") {return sl::RESOLUTION::XVGA;}
  throw std::invalid_argument("unknown resolution: " + value);
}

inline sl::DEPTH_MODE parse_depth_mode(const std::string & value)
{
  if (value == "NEURAL_LIGHT") {return sl::DEPTH_MODE::NEURAL_LIGHT;}
  if (value == "NEURAL") {return sl::DEPTH_MODE::NEURAL;}
  if (value == "NEURAL_PLUS") {return sl::DEPTH_MODE::NEURAL_PLUS;}
  if (value == "PERFORMANCE") {return sl::DEPTH_MODE::PERFORMANCE;}
  if (value == "QUALITY") {return sl::DEPTH_MODE::QUALITY;}
  if (value == "ULTRA") {return sl::DEPTH_MODE::ULTRA;}
  if (value == "NONE") {return sl::DEPTH_MODE::NONE;}
  throw std::invalid_argument("unknown depth mode: " + value);
}

inline sl::SVO_COMPRESSION_MODE parse_svo_compression(const std::string & value)
{
  if (value == "H264") {return sl::SVO_COMPRESSION_MODE::H264;}
  if (value == "H265") {return sl::SVO_COMPRESSION_MODE::H265;}
  if (value == "LOSSLESS") {return sl::SVO_COMPRESSION_MODE::LOSSLESS;}
  if (value == "H264_LOSSLESS") {return sl::SVO_COMPRESSION_MODE::H264_LOSSLESS;}
  if (value == "H265_LOSSLESS") {return sl::SVO_COMPRESSION_MODE::H265_LOSSLESS;}
  throw std::invalid_argument("unknown svo compression: " + value);
}

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
  double baseline_m{0.0};
  unsigned int serial_number{0};
};

class ZedSource
{
public:
  void open(const ZedAcquireConfig & config)
  {
    sl::InitParameters parameters;
    if (!config.input_svo.empty()) {
      parameters.input.setFromSVOFile(config.input_svo.c_str());
      parameters.svo_real_time_mode = config.svo_real_time;
    } else {
      parameters.camera_resolution = parse_resolution(config.resolution);
      parameters.camera_fps = config.fps;
    }
    parameters.depth_mode = parse_depth_mode(config.depth_mode);
    parameters.coordinate_units = sl::UNIT::METER;
    parameters.depth_minimum_distance =
      static_cast<float>(config.depth_min_m);
    parameters.depth_maximum_distance =
      static_cast<float>(config.depth_max_m);
    parameters.open_timeout_sec = 10.0f;
    parameters.async_grab_camera_recovery = true;
    const sl::ERROR_CODE status = camera_.open(parameters);
    if (status != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(
        "ZED open failed: " + std::string(sl::toString(status).c_str()));
    }
    runtime_.confidence_threshold = config.runtime_confidence;
    const sl::CameraInformation information = camera_.getCameraInformation();
    intrinsics_.serial_number = information.serial_number;
    // retrieveImage/retrieveMeasure 输出的是校正后图像,配 calibration_parameters。
    const auto & calibration =
      information.camera_configuration.calibration_parameters;
    intrinsics_.fx = calibration.left_cam.fx;
    intrinsics_.fy = calibration.left_cam.fy;
    intrinsics_.cx = calibration.left_cam.cx;
    intrinsics_.cy = calibration.left_cam.cy;
    intrinsics_.width = information.camera_configuration.resolution.width;
    intrinsics_.height = information.camera_configuration.resolution.height;
    // SDK 5.5:基线单位跟随 InitParameters.coordinate_units(本链路为米)。
    intrinsics_.baseline_m = calibration.getCameraBaseline();
    std::fprintf(
      stderr,
      "[zed_source] opened %s fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d baseline=%.3fm\n",
      config.resolution.c_str(), intrinsics_.fx, intrinsics_.fy, intrinsics_.cx,
      intrinsics_.cy, intrinsics_.width, intrinsics_.height,
      intrinsics_.baseline_m);
  }

  // 抓一帧。返回 false 表示本次 grab 失败(调用方自行计数容错)。
  bool grab() {return camera_.grab(runtime_) == sl::ERROR_CODE::SUCCESS;}

  // 左目 BGRA(全分辨率)。目标 Mat 由本类持有,指针在下次 retrieve 前有效。
  const sl::Mat & retrieve_left()
  {
    check(
      camera_.retrieveImage(left_, sl::VIEW::LEFT, sl::MEM::CPU),
      "retrieveImage LEFT");
    return left_;
  }

  const sl::Mat & retrieve_depth()
  {
    check(
      camera_.retrieveMeasure(depth_, sl::MEASURE::DEPTH, sl::MEM::CPU),
      "retrieveMeasure DEPTH");
    return depth_;
  }

  const sl::Mat & retrieve_confidence()
  {
    check(
      camera_.retrieveMeasure(confidence_, sl::MEASURE::CONFIDENCE, sl::MEM::CPU),
      "retrieveMeasure CONFIDENCE");
    return confidence_;
  }

  // 最近一次成功 grab 的图像时间戳(UTC 纳秒)。SDK 接口非 const。
  std::int64_t image_timestamp_ns()
  {
    return static_cast<std::int64_t>(
      camera_.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns);
  }

  const CameraIntrinsics & intrinsics() const {return intrinsics_;}

  void start_recording(const std::string & path, const std::string & compression)
  {
    sl::RecordingParameters parameters;
    parameters.compression_mode = parse_svo_compression(compression);
    parameters.video_filename = sl::String(path.c_str());
    parameters.bitrate = 0;  // SDK 默认码率
    const auto status = camera_.enableRecording(parameters);
    if (status != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(
        std::string("enableRecording failed: ") + sl::toString(status).c_str());
    }
    recording_.store(true);
  }

  void stop_recording()
  {
    if (recording_.exchange(false)) {
      camera_.disableRecording();
      // SVO 写线程 flush 需要短暂窗口;立即 close 会截断尾部块,
      // 回放时报 "Corrupted message size"(2026-10-07 实测)。
      std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
  }

  bool recording() const {return recording_.load();}

  sl::RecordingStatus recording_status() {return camera_.getRecordingStatus();}

  void close()
  {
    stop_recording();
    camera_.close();
  }

private:
  static void check(sl::ERROR_CODE status, const char * what)
  {
    if (status != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(
        std::string(what) + " failed: " + sl::toString(status).c_str());
    }
  }

  sl::Camera camera_;
  sl::RuntimeParameters runtime_{};
  sl::Mat left_;
  sl::Mat depth_;
  sl::Mat confidence_;
  CameraIntrinsics intrinsics_{};
  std::atomic<bool> recording_{false};
};

}  // namespace zed_tracking
