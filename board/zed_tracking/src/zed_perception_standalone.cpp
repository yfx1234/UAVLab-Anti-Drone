// Perception-only camera owner. No PX4, external poses, or control launch paths.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <openssl/sha.h>
#include <mutex>
#include <thread>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <zed_perception_interfaces/msg/perception_frame.hpp>
#include <zed_perception_interfaces/srv/perception_command.hpp>
#include "zed_tracking/geometry.hpp"
#include "zed_tracking/perception_quality.hpp"
#include "zed_tracking/camera_modes.hpp"
#include "zed_tracking/generic_sky_detector.hpp"
#include "trt_runner.hpp"
#include "zed_source.hpp"

namespace fs=std::filesystem;
using Json=nlohmann::json;
using Steady=std::chrono::steady_clock;
using zed_tracking::Detection;
static double monotonic_s(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
static void atomic_json(const fs::path& path,const Json& j){
  fs::create_directories(path.parent_path());auto tmp=path;tmp+=".tmp";
  {std::ofstream f(tmp);f.exceptions(std::ios::badbit|std::ios::failbit);f<<j.dump();}
  fs::rename(tmp,path);
}
static Json read_json(const fs::path& path){std::ifstream f(path);if(!f)return Json::object();Json j;f>>j;return j;}
static std::string read_line(const char* path){std::ifstream f(path);std::string s;std::getline(f,s);return s;}
static std::string content_sha256(const std::string& text){unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()),text.size(),hash);std::ostringstream out;
  for(unsigned char c:hash)out<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(c);return out.str();}
struct Frame {
  uint32_t sequence{0};int64_t stamp{0};double acquired{0};int w{0},h{0};
  std::vector<unsigned char> bgra;std::vector<float> depth,confidence;
  std::string session;uint32_t segment{0};bool recorded{false};uint64_t detector_generation{0};
  double effective_size{.30};std::string model,calibration_id;bool calibrated{false};
  bool yolo_enabled{false},generic_enabled{false};double nominal_size{.30};
  std::string generic_sensitivity{"standard"};zed_tracking::CameraMotionSample motion;
};
struct PreviewJob {std::shared_ptr<const Frame> frame;Json metadata;};

// Only JPEG work and TCP writes happen here. Latest one-slot queue discards old
// jobs under congestion, and never calls the camera SDK.
class PreviewStream {
  std::atomic<bool> stop_{false};std::thread thread_;std::mutex mutex_;std::condition_variable cv_;
  PreviewJob latest_;int listener_{-1},client_{-1};
  bool send_all(const void* ptr,size_t len){
    auto p=static_cast<const char*>(ptr);
    while(len){ssize_t n=send(client_,p,len,MSG_NOSIGNAL);if(n<=0)return false;p+=n;len-=size_t(n);}return true;
  }
  void loop(){
    listener_=socket(AF_INET,SOCK_STREAM,0);int reuse=1;setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(49543);inet_pton(AF_INET,"192.168.50.100",&addr.sin_addr);
    if(bind(listener_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))||listen(listener_,2))return;
    listening=true;
    double last=0,updated=monotonic_s(),tokens=250000;int quality=70;
    while(!stop_){
      if(client_<0){pollfd p{listener_,POLLIN,0};if(poll(&p,1,100)>0){
        sockaddr_in peer{};socklen_t n=sizeof(peer);client_=accept(listener_,reinterpret_cast<sockaddr*>(&peer),&n);
        if(client_>=0 && peer.sin_addr.s_addr!=inet_addr("192.168.50.101")){close(client_);client_=-1;continue;}
        timeval timeout{0,150000};if(client_>=0)setsockopt(client_,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
      }else continue;}
      PreviewJob job;
      {std::unique_lock<std::mutex> lock(mutex_);cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return stop_||bool(latest_.frame);});
       if(stop_)break;if(!latest_.frame)continue;job=std::move(latest_);latest_=PreviewJob{};}
      double now=monotonic_s();if(now-last<.125)continue;
      cv::Mat bgra(job.frame->h,job.frame->w,CV_8UC4,const_cast<unsigned char*>(job.frame->bgra.data())),small,bgr;
      const int preview_height=std::max(1,int(std::lround(640.0*job.frame->h/job.frame->w)));
      cv::resize(bgra,small,cv::Size(640,preview_height));cv::cvtColor(small,bgr,cv::COLOR_BGRA2BGR);
      std::vector<unsigned char> jpeg;cv::imencode(".jpg",bgr,jpeg,{cv::IMWRITE_JPEG_QUALITY,quality});
      job.metadata["preview_width"]=640;job.metadata["preview_height"]=preview_height;
      job.metadata["source_age_s"]=now-job.frame->acquired;
      std::string meta=job.metadata.dump();size_t bytes=meta.size()+jpeg.size()+8;
      if(meta.size()>65536||jpeg.size()>4194304)continue;
      tokens=std::min(250000.0,tokens+(now-updated)*250000);updated=now;
      if(bytes>tokens){quality=std::max(45,quality-5);continue;}
      tokens-=bytes;uint32_t header[2]={htonl(uint32_t(meta.size())),htonl(uint32_t(jpeg.size()))};
      if(!send_all(header,8)||!send_all(meta.data(),meta.size())||!send_all(jpeg.data(),jpeg.size())){close(client_);client_=-1;}
      else{last=now;++frames_sent;bytes_sent+=bytes;}
    }
  }
public:
  std::atomic<uint64_t> frames_sent{0},bytes_sent{0};
  std::atomic<bool> listening{false};
  PreviewStream(){thread_=std::thread([this]{loop();});}
  ~PreviewStream(){stop();}
  void stop(){if(stop_.exchange(true))return;cv_.notify_all();if(thread_.joinable())thread_.join();
    if(client_>=0){close(client_);client_=-1;}if(listener_>=0){close(listener_);listener_=-1;}listening=false;}
  void offer(std::shared_ptr<const Frame> frame,Json meta){std::lock_guard<std::mutex> lock(mutex_);latest_={std::move(frame),std::move(meta)};cv_.notify_one();}
};

class Perception final:public rclcpp::Node {
  fs::path root_{"/tmp/zed_perception"},records_{"/home/uavlab/Jetson_ROS2_V0/perception_records"};
  fs::path calibration_file_{"/home/uavlab/Jetson_ROS2_V0/perception_config/range_calibration.json"};
  std::string run_,boot_,profile_,engine_,model_;double target_size_{.30},effective_size_{.30};std::string size_definition_{"unconfirmed_30cm_extent"},calibration_id_{"initial_prior"};
  std::atomic<bool> stop_{false},detector_{false};std::atomic<int> detector_state_{0}; // disabled/loading/running/error
  std::atomic<bool> generic_detector_{false};std::atomic<int> generic_state_{0};
  std::string generic_sensitivity_{"standard"};
  std::atomic<uint64_t> detector_generation_{0};bool calibrated_{false};
  std::atomic<uint64_t> config_revision_{0};
  Json calibration_data_;
  std::atomic<double> write_rate_{0};std::atomic<uint64_t> disk_reserve_{10ull*1024*1024*1024};
  std::atomic<bool> metadata_failed_{false};
  int capture_fps_{15};
  std::string camera_resolution_{"HD1080"};
  std::atomic<bool> depth_enabled_{false};
  std::atomic<double> record_started_{0};
  std::atomic<uint32_t> sequence_{0};std::atomic<uint64_t> inferred_{0},dropped_{0};
  std::atomic<double> last_frame_{0},last_infer_{0};double started_{monotonic_s()};
  std::thread capture_,infer_,writer_;std::mutex state_mutex_,queue_mutex_,log_mutex_,command_mutex_,status_io_mutex_;
  std::condition_variable queue_cv_,log_cv_;std::shared_ptr<const Frame> latest_;
  std::deque<std::pair<fs::path,Json>> logs_;std::atomic<uint64_t> dropped_logs_{0};bool writer_stop_{false};
  zed_tracking::SessionWorkCounter pending_work_;uint64_t session_dropped_start_{0};
  std::deque<Json> commands_;int owner_lock_{-1};zed_tracking::CameraIntrinsics intrinsics_;
  Json status_{{"camera_state","starting"},{"record_state","idle"},{"errors",Json::array()}};
  fs::path session_;std::string session_id_;int segment_{0};double segment_start_{0};uint64_t recorded_frames_{0};
  std::vector<Json> segment_summaries_;int recording_failures_{0};
  double last_status_time_{0};uint32_t last_status_capture_{0};uint64_t last_status_infer_{0};
  PreviewStream preview_;
  rclcpp::Publisher<zed_perception_interfaces::msg::PerceptionFrame>::SharedPtr observations_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_,status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_results_;
  rclcpp::Service<zed_perception_interfaces::srv::PerceptionCommand>::SharedPtr command_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  void set_status(const std::string& key,const Json& value){std::lock_guard<std::mutex> l(state_mutex_);status_[key]=value;}
  void apply_range_calibration(){
    std::lock_guard<std::mutex> l(state_mutex_);
    zed_tracking::CalibrationBinding current{model_,intrinsics_.serial_number,intrinsics_.width,intrinsics_.height,intrinsics_.fx,intrinsics_.fy};
    zed_tracking::CalibrationBinding saved{calibration_data_.value("model_id",""),calibration_data_.value("camera_serial",0u),
      calibration_data_.value("image_width",0),calibration_data_.value("image_height",0),calibration_data_.value("fx",0.0),calibration_data_.value("fy",0.0)};
    bool compatible=zed_tracking::calibration_matches(current,saved);
    calibrated_=compatible&&calibration_data_.value("calibrated",false);
    effective_size_=calibrated_?calibration_data_.value("effective_size_m",target_size_):target_size_;
    calibration_id_=compatible?calibration_data_.value("calibration_id","initial_prior"):"initial_prior";
    status_["calibration_compatible"]=compatible;status_["size_estimate_calibrated"]=calibrated_;
  }
  void error(const std::string& module,const std::string& message){
    RCLCPP_ERROR(get_logger(),"%s: %s",module.c_str(),message.c_str());
    std::lock_guard<std::mutex> l(state_mutex_);status_["errors"].push_back({{"module",module},{"message",message}});
    while(status_["errors"].size()>10)status_["errors"].erase(status_["errors"].begin());
  }
  Json base(const Frame& f){
    std::lock_guard<std::mutex> config_lock(state_mutex_);
    Json j={{"schema_version",1},{"run_id",run_},{"boot_id",boot_},{"frame_id","zed_left_camera_optical_frame"},
      {"frame_key",run_+"_"+std::to_string(f.sequence)},{"sequence",f.sequence},{"stamp_ns",f.stamp},
      {"monotonic_s",f.acquired},{"image_width",f.w},{"image_height",f.h},{"model_id",f.model},{"camera_serial",intrinsics_.serial_number},
      {"detector_enabled",(f.yolo_enabled&&detector_state_==2)||(f.generic_enabled&&generic_state_==2)},
      {"yolo_enabled",f.yolo_enabled},{"detector_state",detector_state_.load()},
      {"generic_detector_enabled",f.generic_enabled},{"generic_detector_state",generic_state_.load()},
      {"detector_generation",f.detector_generation},{"generic_sensitivity",f.generic_sensitivity},
      {"camera_motion_valid",f.motion.valid},
      {"session_id",f.session},{"segment_id",f.segment},{"recorded",nullptr},{"record_submitted",f.recorded},{"targets",Json::array()}};
    j["intrinsics"]={{"fx",intrinsics_.fx},{"fy",intrinsics_.fy},{"cx",intrinsics_.cx},{"cy",intrinsics_.cy}};
    return j;
  }
  void log(const fs::path& file,Json value){
    std::lock_guard<std::mutex> l(log_mutex_);
    if(logs_.size()>=1024){++dropped_logs_;metadata_failed_=true;return;}
    pending_work_.acquire(file.parent_path().filename().string());logs_.emplace_back(file,std::move(value));log_cv_.notify_one();
  }
  void writer_loop(){
    while(true){std::pair<fs::path,Json> item;
      {std::unique_lock<std::mutex> l(log_mutex_);log_cv_.wait(l,[&]{return writer_stop_||!logs_.empty();});
       if(logs_.empty()&&writer_stop_)break;item=std::move(logs_.front());logs_.pop_front();}
      try{std::ofstream f(item.first,std::ios::app);f.exceptions(std::ios::badbit|std::ios::failbit);f<<item.second.dump()<<'\n';}
      catch(const std::exception& e){++dropped_logs_;if(!metadata_failed_.exchange(true))error("metadata",e.what());}
      pending_work_.release(item.first.parent_path().filename().string());
    }
  }
  void publish_status(){
    std::lock_guard<std::mutex> status_guard(status_io_mutex_);
    Json j;{std::lock_guard<std::mutex> l(state_mutex_);j=status_;j["model_id"]=model_;j["target_size_m"]=target_size_;j["calibration_id"]=calibration_id_;j["generic_sensitivity"]=generic_sensitivity_;}
    double now=monotonic_s();j["run_id"]=run_;j["boot_id"]=boot_;j["monotonic_s"]=now;
    j["profile"]=profile_;j["depth_enabled"]=depth_enabled_.load();
    j["capture_frames"]=sequence_.load();j["inference_frames"]=inferred_.load();j["dropped_frames"]=dropped_.load();
    j["last_frame_age_s"]=last_frame_>0?Json(now-last_frame_):Json(nullptr);
    j["last_inference_age_s"]=last_infer_>0?Json(now-last_infer_):Json(nullptr);
    double elapsed=std::max(.001,now-started_),window=std::max(.001,now-last_status_time_);
    bool running=j["camera_state"]=="running";
    j["capture_hz"]=running&&last_status_time_>0?double(sequence_-last_status_capture_)/window:0;
    j["inference_hz"]=running&&(detector_||generic_detector_)&&last_status_time_>0?double(inferred_-last_status_infer_)/window:0;
    j["average_capture_hz"]=sequence_/elapsed;j["average_inference_hz"]=inferred_/elapsed;
    last_status_time_=now;last_status_capture_=sequence_;last_status_infer_=inferred_;
    j["detector_state"]=detector_state_.load();j["detector_enabled"]=detector_.load();
    j["generic_detector_state"]=generic_state_.load();j["generic_detector_enabled"]=generic_detector_.load();
    j["detector_generation"]=detector_generation_.load();
    j["preview_frames"]=preview_.frames_sent.load();j["preview_mbps"]=preview_.bytes_sent*8/elapsed/1e6;j["preview_listening"]=preview_.listening.load();
    j["dropped_metadata"]=dropped_logs_.load();
    j["node_config_revision"]=config_revision_.load();
    j["record_write_bytes_s"]=write_rate_.load();j["disk_reserve_bytes"]=disk_reserve_.load();
    j["targets_valid"]=zed_tracking::targets_are_fresh(running,(detector_&&detector_state_==2)||(generic_detector_&&generic_state_==2),
      last_frame_>0?now-last_frame_:-1,last_infer_>0?now-last_infer_:-1);
    j["record_elapsed_s"]=record_started_>0?Json(now-record_started_):Json(nullptr);
    try{const uint64_t available=fs::space(records_.parent_path()).available;j["disk_free_bytes"]=available;
      j["record_remaining_s"]=record_started_>0&&write_rate_>0?Json(double(available>disk_reserve_?available-disk_reserve_.load():0)/write_rate_.load()):Json(nullptr);
      atomic_json(root_/"status.json",j);}
    catch(const std::exception& e){RCLCPP_WARN(get_logger(),"status write: %s",e.what());}
    if(rclcpp::ok()){std_msgs::msg::String msg;msg.data=j.dump();status_pub_->publish(msg);}
  }
  void begin_segment(zed_tracking::ZedSource& source){
    fs::path file=session_/"segments"/("segment_"+std::to_string(++segment_)+".svo2");
    source.start_recording(file.string(),"LOSSLESS");segment_start_=monotonic_s();recording_failures_=0;
    set_status("segment_id",segment_);
  }
  void close_segment(zed_tracking::ZedSource& source){
    auto rs=source.recording_status();source.stop_recording();
    segment_summaries_.push_back({{"segment_id",segment_},{"ingested",rs.number_frames_ingested},{"encoded_before_close",rs.number_frames_encoded},
      {"duration_s",monotonic_s()-segment_start_},{"state","closed_pending_validation"}});
  }
  void finish_record(zed_tracking::ZedSource& source,const std::string& reason="user_stop"){
    if(session_id_.empty())return;set_status("record_state","finalizing");
    close_segment(source);
    const std::string closing_id=session_id_;
    if(stop_){std::lock_guard<std::mutex> l(queue_mutex_);if(latest_&&latest_->session==closing_id){pending_work_.release(latest_->session);latest_.reset();}}
    log(session_/"events.jsonl",{{"event","record_stop"},{"reason",reason},{"monotonic_s",monotonic_s()}});
    bool drained=pending_work_.wait(closing_id,std::chrono::seconds(10));
    atomic_json(session_/"session_summary.json",{{"session_id",session_id_},{"state","closed_pending_validation"},
      {"submitted_frames",recorded_frames_},{"recorded_frames",nullptr},{"segments",segment_summaries_},{"reason",reason},
      {"dropped_metadata",dropped_logs_.load()-session_dropped_start_},{"metadata_complete",drained&&!metadata_failed_},
      {"tail_metadata_may_arrive",!drained}});
    session_id_.clear();session_.clear();record_started_=0;set_status("record_state","idle");set_status("session_id",nullptr);
  }
  void process_command(zed_tracking::ZedSource& source,const Json& cmd){
    std::string rid=cmd.value("request_id","");Json answer={{"ok",false}};
    try{
      if(rid.empty()||rid.size()>80||rid.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)throw std::runtime_error("invalid request ID");
      if(cmd.value("run_id","")!=run_)throw std::runtime_error("run changed");
      if(cmd.value("expected_config_revision",config_revision_.load())!=config_revision_)throw std::runtime_error("node configuration changed");
      std::string action=cmd.at("operation");Json p=cmd.value("params",Json::object());
      if(action=="set_detector"){
        bool enabled=p.at("enabled");
        if(enabled&&!zed_tracking::model_change_allowed(!session_id_.empty(),model_,p.value("model_id",model_)))
          throw std::runtime_error("finish recording before changing model identity");
        if(enabled){{std::lock_guard<std::mutex> l(state_mutex_);engine_=p.value("engine_path",engine_);model_=p.value("model_id",model_);}apply_range_calibration();}
        source.set_depth_enabled(enabled||generic_detector_);depth_enabled_=source.depth_enabled();
        ++detector_generation_;detector_=enabled;detector_state_=enabled?1:0;
        {std::lock_guard<std::mutex> l(queue_mutex_);if(latest_)pending_work_.release(latest_->session);latest_.reset();}queue_cv_.notify_all();
        if(!session_id_.empty())log(session_/"events.jsonl",{{"event","detector_change"},{"source","yolo"},{"enabled",enabled},{"generation",detector_generation_.load()},{"monotonic_s",monotonic_s()}});
      }else if(action=="set_generic_detector"){
        bool enabled=p.at("enabled");std::string sensitivity;
        {std::lock_guard<std::mutex> l(state_mutex_);sensitivity=p.value("sensitivity",generic_sensitivity_);}
        zed_tracking::sky_options(sensitivity);
        source.set_depth_enabled(enabled||detector_);depth_enabled_=source.depth_enabled();
        {std::lock_guard<std::mutex> l(state_mutex_);generic_sensitivity_=sensitivity;}
        ++detector_generation_;generic_detector_=enabled;generic_state_=enabled?1:0;
        {std::lock_guard<std::mutex> l(queue_mutex_);if(latest_)pending_work_.release(latest_->session);latest_.reset();}queue_cv_.notify_all();
        if(!session_id_.empty())log(session_/"events.jsonl",{{"event","detector_change"},{"source","generic"},{"enabled",enabled},{"sensitivity",sensitivity},{"generation",detector_generation_.load()},{"monotonic_s",monotonic_s()}});
      }else if(action=="start_raw_record"){
        if(!session_id_.empty())throw std::runtime_error("recording already active");
        if(fs::space(records_.parent_path()).available<disk_reserve_)throw std::runtime_error("disk below measured recording reserve");
        if(metadata_failed_)throw std::runtime_error("metadata writer failed; restart and check storage before recording");
        session_id_="session_"+read_line("/proc/sys/kernel/random/uuid");session_=records_/session_id_;
        if(fs::exists(session_))throw std::runtime_error("session collision; existing recordings are never overwritten");
        fs::create_directories(session_/"segments");
        segment_=0;recorded_frames_=0;segment_summaries_.clear();session_dropped_start_=dropped_logs_;
        Json manifest={{"schema_version",1},{"session_id",session_id_},{"request_id",rid},{"run_id",run_},{"boot_id",boot_},
          {"profile",profile_},{"codec","LOSSLESS"},{"segment_seconds",60},{"model_id",model_},
          {"camera_resolution",camera_resolution_},{"configured_capture_fps",capture_fps_},
          {"yolo_enabled",detector_.load()},{"generic_detector_enabled",generic_detector_.load()},{"generic_sensitivity",generic_sensitivity_},
          {"generic_size_prior_calibrated",false},{"generic_size_prior_m",target_size_},
          {"camera_serial",intrinsics_.serial_number},{"sdk_version",std::string(sl::Camera::getSDKVersion().c_str())},
          {"tensorrt_version",std::to_string(NV_TENSORRT_MAJOR)+"."+std::to_string(NV_TENSORRT_MINOR)},
          {"target_size_m",target_size_},{"calibration_id",calibration_id_},{"metadata",p},
          {"intrinsics",{{"fx",intrinsics_.fx},{"fy",intrinsics_.fy},{"cx",intrinsics_.cx},{"cy",intrinsics_.cy},
                         {"width",intrinsics_.width},{"height",intrinsics_.height},{"baseline_m",intrinsics_.baseline_m}}},
          {"warnings",{"recording rotation may create a gap; use timestamp mapping","model recognition and physical-center accuracy not field validated"}}};
        const double board_wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        double created=board_wall;std::string time_source="board_clock";
        const auto anchor=read_json(root_/"clock_reference.json");
        if(anchor.value("boot_id",std::string{})==boot_&&anchor.contains("pc_unix_s")&&anchor.contains("board_monotonic_s")){
          created=anchor["pc_unix_s"].get<double>()+monotonic_s()-anchor["board_monotonic_s"].get<double>();time_source="pc_reference_monotonic";
        }
        manifest["created_at_unix_s"]=created;manifest["created_time_source"]=time_source;
        manifest["created_board_unix_s"]=board_wall;manifest["created_monotonic_s"]=monotonic_s();
        Json snapshot={{"target_size_m",target_size_},{"effective_size_m",effective_size_},
          {"calibration_id",calibration_id_},{"size_definition",size_definition_},{"calibrated",calibrated_},
          {"model_id",model_},{"camera_serial",intrinsics_.serial_number},{"image_width",intrinsics_.width},{"image_height",intrinsics_.height},
          {"fx",intrinsics_.fx},{"fy",intrinsics_.fy},{"saved_calibration",calibration_data_}};
        manifest["calibration_sha256"]=content_sha256(snapshot.dump());
        atomic_json(session_/"manifest.json",manifest);atomic_json(session_/"calibration.json",snapshot);
        try{begin_segment(source);}catch(...){session_id_.clear();session_.clear();throw;}
        atomic_json(session_/"session_summary.json",{{"state","recording"},{"session_id",session_id_}});
        record_started_=monotonic_s();set_status("record_state","recording");set_status("session_id",session_id_);answer["session_id"]=session_id_;
      }else if(action=="stop_raw_record")finish_record(source);
      else if(action=="save_calibration"){
        if(!session_id_.empty())throw std::runtime_error("finish recording before calibration change");
        double size=p.at("target_size_m");if(!std::isfinite(size)||size<=0||size>5)throw std::runtime_error("invalid size");
        auto samples=p.value("samples",Json::array());std::vector<double> fitted;
        for(const auto& s:samples){double z=s.at("known_distance_m"),bw=s.at("bbox_width"),bh=s.at("bbox_height"),fx=s.at("fx"),fy=s.at("fy");
          if(!std::isfinite(z+bw+bh+fx+fy)||z<=0||bw<8||bh<=0||fx<=0||fy<=0)throw std::runtime_error("invalid calibration sample");
          if(s.value("model_id","")!=model_||s.value("camera_serial",0u)!=intrinsics_.serial_number||
             s.value("image_width",0)!=intrinsics_.width||s.value("image_height",0)!=intrinsics_.height||
             std::abs(fx-intrinsics_.fx)/intrinsics_.fx>.01||std::abs(fy-intrinsics_.fy)/intrinsics_.fy>.01)
            throw std::runtime_error("sample belongs to another model/camera/intrinsics");
          fitted.push_back(z*std::max(bw/fx,bh/fy));}
        if(!fitted.empty()&&fitted.size()<3)throw std::runtime_error("need at least 3 measured samples");
        std::sort(fitted.begin(),fitted.end());double extent=fitted.empty()?size:fitted[fitted.size()/2];
        if(extent<size*.05||extent>size*2)throw std::runtime_error("implausible projection extent; check target and measured samples");
        Json proposed={{"target_size_m",size},{"effective_size_m",extent},{"size_definition",p.value("size_definition","unconfirmed_extent")},
          {"calibration_id","cal_"+read_line("/proc/sys/kernel/random/uuid")},{"calibrated",fitted.size()>=3},{"samples",samples},
          {"model_id",model_},{"camera_serial",intrinsics_.serial_number},{"image_width",intrinsics_.width},{"image_height",intrinsics_.height},
          {"fx",intrinsics_.fx},{"fy",intrinsics_.fy}};
        atomic_json(calibration_file_,proposed);
        {std::lock_guard<std::mutex> l(state_mutex_);target_size_=size;size_definition_=proposed["size_definition"];calibration_data_=proposed;}
        apply_range_calibration();
      }else throw std::runtime_error("unsupported node operation");
      answer["ok"]=true;answer["node_config_revision"]=++config_revision_;
      atomic_json(root_/"node_revision.json",{{"run_id",run_},{"node_config_revision",config_revision_.load()}});
    }catch(const std::exception& e){answer["error"]=e.what();error("command",e.what());}
    // Publish authoritative lifecycle state before acknowledging a command.
    // Otherwise an immediate model switch can see the previous idle snapshot.
    publish_status();
    if(!rid.empty()&&rid.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos)
      atomic_json(root_/"responses"/(rid+".json"),answer);
    if(rclcpp::ok()){answer["operation_id"]=rid;std_msgs::msg::String msg;msg.data=answer.dump();command_results_->publish(msg);}
  }
  void commands(zed_tracking::ZedSource& source){
    fs::path path=root_/"command.json";
    if(fs::exists(path)){
      try{Json cmd=read_json(path);fs::remove(path);process_command(source,cmd);}catch(const std::exception& e){error("command_file",e.what());}
    }
    std::deque<Json> batch;{std::lock_guard<std::mutex> l(command_mutex_);batch.swap(commands_);}
    for(const auto& cmd:batch)process_command(source,cmd);
  }
  void capture_loop(){
    zed_tracking::ZedSource source;
    try{
      zed_tracking::ZedAcquireConfig cfg;cfg.fps=capture_fps_;cfg.resolution=camera_resolution_;
      cfg.depth_mode="NEURAL_LIGHT";cfg.depth_computation_enabled=detector_||generic_detector_;source.open(cfg);
      if(std::abs(source.configured_fps()-capture_fps_)>.5)throw std::runtime_error("camera FPS differs from requested configuration");
      depth_enabled_=source.depth_enabled();
      intrinsics_=source.intrinsics();apply_range_calibration();set_status("camera_state","running");
      set_status("configured_capture_fps",cfg.fps);
      set_status("camera_resolution",cfg.resolution);set_status("image_width",intrinsics_.width);set_status("image_height",intrinsics_.height);
      int failures=0;double last_resource=0;uint64_t previous_free=fs::space(records_.parent_path()).available;
      while(!stop_&&rclcpp::ok()){
        commands(source);
        bool any_detector=detector_||generic_detector_;
        if(source.depth_enabled()!=any_detector){source.set_depth_enabled(any_detector);depth_enabled_=source.depth_enabled();}
        if(!source.grab()){
          if(++failures>=40)throw std::runtime_error("repeated SDK grab failure");
          std::this_thread::sleep_for(std::chrono::milliseconds(50));continue;
        }
        failures=0;uint32_t seq=++sequence_;double now=monotonic_s();last_frame_=now;
        bool recorded=false;int64_t stamp=source.image_timestamp_ns();
        auto motion=source.camera_motion(stamp);
        std::string sid=session_id_;int seg=segment_;
        if(!sid.empty()){
          auto rs=source.recording_status();recorded=true;++recorded_frames_;
          if(rs.status)recording_failures_=0;else ++recording_failures_;
          log(session_/"frames.jsonl",{{"sequence",seq},{"stamp_ns",stamp},{"monotonic_s",now},{"segment_id",seg},
            {"submitted",true},{"recorded",nullptr},{"sdk_recording_status",rs.status},
            {"sdk_frames_ingested",rs.number_frames_ingested},{"sdk_frames_encoded",rs.number_frames_encoded},
            {"yolo_enabled",detector_.load()},{"generic_detector_enabled",generic_detector_.load()},
            {"detector_generation",detector_generation_.load()},{"camera_motion_valid",motion.valid},
            {"camera_motion_timestamp_ns",motion.timestamp_ns},{"camera_quaternion_wxyz",motion.quaternion_wxyz}});
          if(recording_failures_>=10)finish_record(source,"sdk_recording_failed");
          else if(metadata_failed_)finish_record(source,"metadata_writer_failed");
          else if(fs::space(records_.parent_path()).available<disk_reserve_)finish_record(source,"disk_reserve");
          else if(now-segment_start_>=60){close_segment(source);begin_segment(source);}
        }
        if(sid!=session_id_){sid.clear();recorded=false;}
        if(now-last_resource>=1){double resource_window=last_resource>0?now-last_resource:1;last_resource=now;
          uint64_t free=fs::space(records_.parent_path()).available;
          if(!session_id_.empty()&&previous_free>free){double rate=double(previous_free-free)/resource_window;write_rate_=std::max(rate,write_rate_.load()*.95);disk_reserve_=zed_tracking::disk_reserve_bytes(write_rate_);}
          previous_free=free;
          double max_temp=0;
          for(const auto& zone:fs::directory_iterator("/sys/class/thermal")){
            if(zone.path().filename().string().rfind("thermal_zone",0)!=0)continue;
            std::ifstream file(zone.path()/"temp");double milli=0;file>>milli;if(milli>1000&&milli<200000)max_temp=std::max(max_temp,milli/1000);
          }
          set_status("max_temperature_c",max_temp);
          std::ifstream memory("/proc/meminfo");std::string key,unit;uint64_t val,available=0;
          while(memory>>key>>val>>unit)if(key=="MemAvailable:")available=val;
          set_status("mem_available_kib",available);
          if(!sid.empty())log(records_/sid/"resources.jsonl",{{"monotonic_s",now},{"mem_available_kib",available},{"disk_free_bytes",free},{"max_temperature_c",max_temp}});
          if(available&&available<1500000)throw std::runtime_error("memory safety reserve reached");
          if(max_temp>=80)throw std::runtime_error("conservative temperature guard 80 C reached");
        }
        // SDK records every grab; FPS is separately selected from YOLO input size.
        auto f=std::make_shared<Frame>();f->sequence=seq;f->stamp=stamp;f->acquired=now;f->session=sid;f->segment=seg;f->recorded=recorded;f->detector_generation=detector_generation_;
        {std::lock_guard<std::mutex> l(state_mutex_);f->model=model_;f->effective_size=effective_size_;f->calibration_id=calibration_id_;f->calibrated=calibrated_;
          f->generic_sensitivity=generic_sensitivity_;f->nominal_size=target_size_;}
        f->yolo_enabled=detector_;f->generic_enabled=generic_detector_;f->motion=motion;
        const sl::Mat& left=source.retrieve_left();f->w=int(left.getWidth());f->h=int(left.getHeight());
        size_t rowbytes=size_t(f->w)*4;f->bgra.resize(rowbytes*f->h);auto ptr=left.getPtr<sl::uchar1>();
        for(int y=0;y<f->h;++y)std::memcpy(f->bgra.data()+y*rowbytes,reinterpret_cast<const unsigned char*>(ptr)+y*left.getStepBytes(),rowbytes);
        if(depth_enabled_){
          try{const auto& d=source.retrieve_depth();const auto& c=source.retrieve_confidence();
            f->depth.resize(size_t(f->w)*f->h);f->confidence.resize(f->depth.size());
            for(int y=0;y<f->h;++y){
              std::memcpy(f->depth.data()+y*f->w,reinterpret_cast<const char*>(d.getPtr<float>())+y*d.getStepBytes(),f->w*sizeof(float));
              std::memcpy(f->confidence.data()+y*f->w,reinterpret_cast<const char*>(c.getPtr<float>())+y*c.getStepBytes(),f->w*sizeof(float));}
          }catch(const std::exception& e){f->depth.clear();f->confidence.clear();if(seq%30==0)error("depth",e.what());}
        }
        if(!f->yolo_enabled&&!f->generic_enabled){preview_.offer(f,base(*f));continue;}
        if((f->yolo_enabled&&detector_state_!=2)||(f->generic_enabled&&generic_state_!=2))preview_.offer(f,base(*f));
        {std::lock_guard<std::mutex> l(queue_mutex_);if(latest_){++dropped_;pending_work_.release(latest_->session);}
          pending_work_.acquire(f->session);latest_=std::move(f);}queue_cv_.notify_one();
      }
      set_status("camera_state","stopping");finish_record(source,"capture_stop");source.close();depth_enabled_=false;set_status("camera_state","stopped");
    }catch(const std::exception& e){error("camera",e.what());set_status("camera_state","error");
      try{finish_record(source,"camera_error");source.close();}catch(const std::exception& close_error){error("record_finalize",close_error.what());}}
    stop_=true;queue_cv_.notify_all();preview_.stop();publish_status();
  }
  std::vector<Detection> detect(const Frame& f,zed_tracking::TrtRunner& runner){
    int iw=runner.input_width(),ih=runner.input_height();cv::Mat src(f.h,f.w,CV_8UC4,const_cast<unsigned char*>(f.bgra.data()));
    auto lb=zed_tracking::make_letterbox(f.w,f.h,iw,ih);int sw=int(std::lround(f.w*lb.scale)),sh=int(std::lround(f.h*lb.scale));
    cv::Mat scaled,padded;cv::resize(src,scaled,cv::Size(sw,sh));
    cv::copyMakeBorder(scaled,padded,lb.pad_y,ih-sh-lb.pad_y,lb.pad_x,iw-sw-lb.pad_x,cv::BORDER_CONSTANT,cv::Scalar(114,114,114,255));
    size_t plane=size_t(iw)*ih;std::vector<float> input(plane*3);
    for(int y=0;y<ih;++y){auto row=padded.ptr<cv::Vec4b>(y);for(int x=0;x<iw;++x){size_t at=size_t(y)*iw+x;
      input[at]=row[x][2]/255.f;input[at+plane]=row[x][1]/255.f;input[at+2*plane]=row[x][0]/255.f;}}
    const auto& output=runner.infer(input.data());const auto& shape=runner.output_shape();
    if(shape.size()!=3||shape[0]!=1||shape[2]!=6)throw std::runtime_error("only end-to-end [1,N,6] output approved");
    auto boxes=zed_tracking::decode_end_to_end(output.data(),shape[1],.55);
    boxes.erase(std::remove_if(boxes.begin(),boxes.end(),[](const Detection& b){return b.class_id!=0||!std::isfinite(b.x1+b.x2+b.y1+b.y2+b.score);}),boxes.end());
    for(auto& b:boxes){double x1=b.x1,y1=b.y1,x2=b.x2,y2=b.y2;zed_tracking::map_box_back(lb,&x1,&y1,&x2,&y2);
      b.x1=float(x1);b.y1=float(y1);b.x2=float(x2);b.y2=float(y2);}
    boxes.erase(std::remove_if(boxes.begin(),boxes.end(),[&](const Detection& b){return b.x2<=0||b.y2<=0||b.x1>=f.w||b.y1>=f.h;}),boxes.end());
    std::sort(boxes.begin(),boxes.end(),[](const Detection& a,const Detection& b){return a.score>b.score;});
    if(boxes.size()>32)boxes.resize(32); // Keeps target metadata and CPU work bounded.
    return boxes;
  }
  void infer_loop(){
    std::unique_ptr<zed_tracking::TrtRunner> runner;zed_tracking::TargetAssociation association;
    zed_tracking::GenericSkyDetector sky_detector;zed_tracking::GenericSkyTracker sky_tracker;
    uint64_t applied_generation=0;
    while(!stop_){std::shared_ptr<const Frame> f;
      {std::unique_lock<std::mutex> l(queue_mutex_);queue_cv_.wait_for(l,std::chrono::milliseconds(100),[&]{return stop_||bool(latest_);});
       if(stop_)break;if(latest_)f=std::move(latest_);}
      if(!detector_)runner.reset(); // Release GPU memory even when both modes are now disabled.
      if(!f)continue;
      auto release_work=[this](const Frame* frame){pending_work_.release(frame->session);};
      std::unique_ptr<const Frame,decltype(release_work)> work_guard(f.get(),release_work);
      if(f->detector_generation!=detector_generation_)continue;
      if(applied_generation!=f->detector_generation){association.reset();sky_tracker.clear();applied_generation=f->detector_generation;}
      try{
        std::vector<Detection> boxes;std::vector<zed_tracking::SkyTrack> generic;
        std::vector<zed_tracking::SkyCandidate> candidates;
        double yolo_ms=0,generic_ms=0;
        if(f->yolo_enabled){
          const auto start=Steady::now();
          try{
            if(!runner){detector_state_=1;runner=std::make_unique<zed_tracking::TrtRunner>();std::string engine;
              {std::lock_guard<std::mutex> l(state_mutex_);engine=engine_;}runner->load(engine);}
            boxes=detect(*f,*runner);
            if(f->detector_generation==detector_generation_)detector_state_=2;
          }catch(const std::exception& e){runner.reset();
            if(f->detector_generation==detector_generation_){detector_=false;detector_state_=3;++detector_generation_;error("yolo",e.what());}}
          yolo_ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
        }
        if(f->generic_enabled && f->detector_generation==detector_generation_){
          const auto start=Steady::now();
          try{
            cv::Mat image(f->h,f->w,CV_8UC4,const_cast<unsigned char*>(f->bgra.data()));
            auto options=zed_tracking::sky_options(f->generic_sensitivity);
            candidates=sky_detector.detect(image,options);
            zed_tracking::SkyIntrinsics k{intrinsics_.fx,intrinsics_.fy,intrinsics_.cx,intrinsics_.cy,f->w,f->h};
            generic=sky_tracker.update(candidates,k,f->stamp*1e-9,f->motion,options.confirmation_frames);
            if(f->detector_generation==detector_generation_)generic_state_=2;
          }catch(const std::exception& e){sky_tracker.clear();
            if(f->detector_generation==detector_generation_){generic_detector_=false;generic_state_=3;++detector_generation_;error("generic",e.what());}}
          generic_ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
        }
        if(stop_||!rclcpp::ok())break;
        if(f->detector_generation!=detector_generation_)continue;
        set_status("yolo_detector_ms",yolo_ms);set_status("generic_detector_ms",generic_ms);
        set_status("camera_motion_valid",f->motion.valid);
        auto ids=association.assign(boxes,f->acquired);Json packet=base(*f);
        struct Input {Detection box;uint32_t id;bool generic_only;std::string source;bool motion_compensated;};
        std::vector<Input> inputs;std::vector<bool> used_generic(generic.size(),false);
        for(size_t i=0;i<boxes.size();++i){bool supported=false;const auto& b=boxes[i];
          zed_tracking::SkyCandidate y{b.x1,b.y1,b.x2,b.y2,b.score};
          for(size_t gi=0;gi<generic.size();++gi){const auto& g=generic[gi];
            const double cx=(g.x1+g.x2)/2,cy=(g.y1+g.y2)/2;
            const double ratio=std::max(g.x2-g.x1,g.y2-g.y1)/std::max(b.x2-b.x1,b.y2-b.y1);
            if(zed_tracking::sky_iou(y,g)>.2 || (ratio>.3&&ratio<2&&cx>=b.x1&&cx<=b.x2&&cy>=b.y1&&cy<=b.y2)){
              used_generic[gi]=true;supported=true;}}
          inputs.push_back({b,ids[i],false,supported?"yolo+generic":"yolo",false});
        }
        for(size_t gi=0;gi<generic.size();++gi)if(!used_generic[gi]){
          const auto& g=generic[gi];Detection b{g.x1,g.y1,g.x2,g.y2,g.score,-1};
          inputs.push_back({b,0x80000000u|g.id,true,"generic",g.motion_compensated});
        }
        packet["detector_timings_ms"]={{"yolo",yolo_ms},{"generic",generic_ms}};
        packet["generic_candidate_count"]=candidates.size();packet["generic_confirmed_count"]=generic.size();
        zed_perception_interfaces::msg::PerceptionFrame msg;
        msg.header.stamp=rclcpp::Time(f->stamp);msg.header.frame_id="zed_left_camera_optical_frame";
        msg.run_id=run_;msg.model_id=packet["model_id"];msg.sequence=f->sequence;msg.image_width=f->w;msg.image_height=f->h;msg.detector_enabled=true;
        msg.yolo_enabled=f->yolo_enabled;msg.generic_detector_enabled=f->generic_enabled;msg.detector_generation=f->detector_generation;
        for(const auto& input:inputs){const auto& b=input.box;double u=(b.x1+b.x2)/2,v=(b.y1+b.y2)/2;
          const double extent=input.generic_only?f->nominal_size:f->effective_size;
          const std::string cid=input.generic_only?"generic_nominal_extent_prior":f->calibration_id;
          const bool calibrated=!input.generic_only&&f->calibrated;
          bool clipped=b.x1<=1||b.y1<=1||b.x2>=f->w-2||b.y2>=f->h-2;
          auto q=zed_tracking::assess_target_depth(f->depth.empty()?nullptr:f->depth.data(),f->confidence.empty()?nullptr:f->confidence.data(),f->w,f->h,b.x1,b.y1,b.x2,b.y2);
          auto estimate=zed_tracking::estimate_target_z(extent,intrinsics_.fx,intrinsics_.fy,b.x2-b.x1,b.y2-b.y1,clipped);
          bool consistent=estimate.valid&&zed_tracking::depth_matches_size_prior(q.z,estimate.z);
          bool separated=q.accepted&&zed_tracking::target_separates_from_background(f->depth.data(),f->confidence.data(),f->w,f->h,b.x1,b.y1,b.x2,b.y2,q.z);
          const bool accepted=q.accepted&&!clipped&&consistent&&separated;
          bool measured=input.generic_only?sky_tracker.confirm_depth(input.id&0x7fffffffu,accepted):association.depth_ready(input.id,accepted);
          bool valid=measured||estimate.valid;double z=measured?q.z:(estimate.valid?estimate.z:0);
          auto p=zed_tracking::project_target(u,v,z,intrinsics_.fx,intrinsics_.fy,intrinsics_.cx,intrinsics_.cy);
          std::string mode=measured?"depth":(valid?"size_estimate":"bearing_only");
          std::string reason=measured?q.reason:(clipped?"clipped_box":q.reason);
          if(q.accepted&&!measured&&!clipped)reason="depth_temporal_warmup";
          if(q.accepted&&!consistent&&!clipped&&estimate.valid)reason="depth_size_disagreement";
          if(q.accepted&&consistent&&!separated&&!clipped)reason="background_depth_ambiguous";
          if(!valid&&!clipped)reason="box_too_small";
          Json target={{"track_id",input.id},{"class_id",b.class_id},{"confidence",b.score},{"bbox_xyxy",{b.x1,b.y1,b.x2,b.y2}},
            {"detector_source",input.source},{"score_kind",input.generic_only?"contrast_score":"class_confidence"},{"class_confirmed",!input.generic_only},
            {"camera_motion_compensated",input.motion_compensated},{"size_prior_m",extent},
            {"pixel_u",u},{"pixel_v",v},{"unit_los",p.los},{"horizontal_angle_rad",p.horizontal},{"vertical_angle_rad",p.vertical},
            {"mode",mode},{"position_valid",valid},{"depth_measured_valid",measured},{"covariance_valid",false},
            {"depth_z_m",measured?Json(z):Json(nullptr)},{"estimated_depth_z_m",!measured&&valid?Json(z):Json(nullptr)},
            {"distance_m",valid?Json(p.distance):Json(nullptr)},{"position_camera",valid?Json({p.x,p.y,p.z}):Json(nullptr)},
            {"depth_valid_pixels",q.count},{"depth_valid_ratio",q.ratio},{"depth_iqr_m",q.iqr},{"reason",reason},
            {"calibration_id",cid},{"size_estimate_calibrated",calibrated},{"position_definition","supported_surface_center_ray"}};
          packet["targets"].push_back(target);
          zed_perception_interfaces::msg::TargetObservation t;t.track_id=input.id;t.class_id=b.class_id;t.confidence=b.score;
          t.detector_source=input.source;t.score_kind=input.generic_only?"contrast_score":"class_confidence";t.class_confirmed=!input.generic_only;
          t.bbox_xyxy={b.x1,b.y1,b.x2,b.y2};t.pixel_u=u;t.pixel_v=v;t.unit_los.x=p.los[0];t.unit_los.y=p.los[1];t.unit_los.z=p.los[2];
          t.horizontal_angle_rad=p.horizontal;t.vertical_angle_rad=p.vertical;t.mode=mode;t.position_valid=valid;t.depth_measured_valid=measured;
          t.depth_z_m=measured?z:0;t.estimated_depth_z_m=!measured&&valid?z:0;t.distance_m=valid?p.distance:0;
          t.position_camera.x=p.x;t.position_camera.y=p.y;t.position_camera.z=p.z;t.depth_valid_pixels=q.count;
          t.depth_valid_ratio=q.ratio;t.depth_iqr_m=q.iqr;t.reason=reason;msg.targets.push_back(t);
        }
        observations_->publish(msg);std_msgs::msg::String out;out.data=packet.dump();json_pub_->publish(out);
        if(!f->session.empty()){
          log(records_/f->session/"observations.jsonl",packet);
          Json comparison={{"sequence",f->sequence},{"stamp_ns",f->stamp},{"detector_generation",f->detector_generation},
            {"yolo_enabled",f->yolo_enabled},{"generic_detector_enabled",f->generic_enabled},{"generic_sensitivity",f->generic_sensitivity},
            {"camera_motion_valid",f->motion.valid},{"timings_ms",packet["detector_timings_ms"]},{"yolo_boxes",Json::array()},
            {"generic_candidates",Json::array()},{"generic_tracks",Json::array()}};
          for(const auto& b:boxes)comparison["yolo_boxes"].push_back({b.x1,b.y1,b.x2,b.y2,b.score});
          for(const auto& b:candidates)comparison["generic_candidates"].push_back({b.x1,b.y1,b.x2,b.y2,b.score});
          for(const auto& b:generic)comparison["generic_tracks"].push_back({{"id",b.id},{"bbox",{b.x1,b.y1,b.x2,b.y2}},{"motion_compensated",b.motion_compensated}});
          log(records_/f->session/"detector_comparison.jsonl",comparison);
        }
        ++inferred_;last_infer_=monotonic_s();preview_.offer(f,std::move(packet));
      }catch(const std::exception& e){if(stop_||!rclcpp::ok())break;error("observations",e.what());preview_.offer(f,base(*f));}
    }
  }
public:
  Perception():Node("zed_perception_standalone"){
    run_=declare_parameter<std::string>("run_id",std::to_string(getpid()));boot_=read_line("/proc/sys/kernel/random/boot_id");
    profile_=declare_parameter<std::string>("profile","rgb");engine_=declare_parameter<std::string>("engine_path","");model_=declare_parameter<std::string>("model_id","");
    capture_fps_=declare_parameter<int>("capture_fps",15);
    camera_resolution_=declare_parameter<std::string>("camera_resolution","HD1080");
    if(!zed_tracking::camera_mode_supported(camera_resolution_,capture_fps_))throw std::runtime_error("unsupported ZED X Mini resolution/FPS pair");
    if(profile_!="rgb"&&profile_!="perception")throw std::runtime_error("invalid profile");
    fs::create_directories(root_);fs::create_directories(records_);
    owner_lock_=::open((root_/"camera.lock").c_str(),O_CREAT|O_RDWR,0600);
    if(owner_lock_<0||flock(owner_lock_,LOCK_EX|LOCK_NB))throw std::runtime_error("camera already owned");
    auto cal=read_json(calibration_file_);calibration_data_=cal;target_size_=cal.value("target_size_m",.30);effective_size_=target_size_;
    size_definition_=cal.value("size_definition",size_definition_);calibration_id_=cal.value("calibration_id",calibration_id_);
    calibrated_=cal.value("calibrated",false);
    detector_=profile_=="perception";detector_state_=detector_?1:0;
    observations_=create_publisher<zed_perception_interfaces::msg::PerceptionFrame>("/perception/observations",rclcpp::SensorDataQoS());
    json_pub_=create_publisher<std_msgs::msg::String>("/perception/observations_json",rclcpp::SensorDataQoS());
    status_pub_=create_publisher<std_msgs::msg::String>("/perception/status",10);
    command_results_=create_publisher<std_msgs::msg::String>("/perception/command_results",10);
    atomic_json(root_/"node_revision.json",{{"run_id",run_},{"node_config_revision",0}});
    command_srv_=create_service<zed_perception_interfaces::srv::PerceptionCommand>("/perception/command",[this](
        const std::shared_ptr<zed_perception_interfaces::srv::PerceptionCommand::Request> req,
        std::shared_ptr<zed_perception_interfaces::srv::PerceptionCommand::Response> res){
      res->operation_id=req->request_id;
      try{if(req->expected_boot_id!=boot_||req->expected_run_id!=run_)throw std::runtime_error("run/boot changed");
        if(req->request_id.empty()||req->request_id.size()>80||req->request_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos||req->parameters_json.size()>65536)throw std::runtime_error("invalid command size/ID");
        if(req->expected_revision!=config_revision_)throw std::runtime_error("node configuration changed; supervisor revision is separate");
        res->revision=config_revision_;
        Json cmd={{"request_id",req->request_id},{"run_id",run_},{"expected_config_revision",req->expected_revision},{"operation",req->operation},{"params",Json::parse(req->parameters_json)}};
        std::lock_guard<std::mutex> l(command_mutex_);if(commands_.size()>=16)throw std::runtime_error("command queue full");commands_.push_back(cmd);res->accepted=true;
      }catch(const std::exception& e){res->accepted=false;res->error_code="invalid";res->message=e.what();}});
    writer_=std::thread([this]{writer_loop();});infer_=std::thread([this]{infer_loop();});capture_=std::thread([this]{capture_loop();});
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{publish_status();});
  }
  ~Perception()override{
    stop_=true;queue_cv_.notify_all();if(capture_.joinable())capture_.join();if(infer_.joinable())infer_.join();
    {std::lock_guard<std::mutex> l(log_mutex_);writer_stop_=true;}log_cv_.notify_all();if(writer_.joinable())writer_.join();
    if(owner_lock_>=0){flock(owner_lock_,LOCK_UN);close(owner_lock_);}publish_status();
  }
};

int main(int argc,char** argv){
  try{
    if(argc==3&&std::string(argv[1])=="--check-model"){
      zed_tracking::TrtRunner r;r.load(argv[2]);auto s=r.output_shape();
      if(s.size()!=3||s[0]!=1||s[2]!=6)throw std::runtime_error("only end-to-end output supported");
      std::vector<float> input(size_t(r.input_width())*r.input_height()*3,0);
      for(int i=0;i<3;++i)r.infer(input.data());
      std::cout<<Json({{"input_nchw",r.input_shape()},{"output_shape",s},{"dtype","float32"},{"contract_valid",true}}).dump()<<std::endl;return 0;
    }
    rclcpp::init(argc,argv);auto node=std::make_shared<Perception>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
