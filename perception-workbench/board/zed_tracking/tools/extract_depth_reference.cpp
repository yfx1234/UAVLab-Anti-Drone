// Offline dataset audit. No model inference, calibration writes or camera controls.
#include <sl/Camera.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <chrono>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>
#include "zed_tracking/perception_quality.hpp"
#include "zed_tracking/geometry.hpp"
#include "trt_runner.hpp"

namespace fs=std::filesystem;
using Json=nlohmann::json;

std::string engine_sha256(const fs::path& path){
  std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("engine missing");
  SHA256_CTX state;SHA256_Init(&state);std::array<char,1024*1024> buffer;
  while(file){file.read(buffer.data(),buffer.size());SHA256_Update(&state,buffer.data(),file.gcount());}
  if(!file.eof())throw std::runtime_error("engine read failed");
  unsigned char digest[SHA256_DIGEST_LENGTH];SHA256_Final(digest,&state);std::ostringstream text;
  for(auto byte:digest)text<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(byte);
  return text.str();
}

Json current_boxes(const cv::Mat& source,zed_tracking::TrtRunner& runner){
  int iw=runner.input_width(),ih=runner.input_height();
  auto lb=zed_tracking::make_letterbox(source.cols,source.rows,iw,ih);
  int sw=int(std::lround(source.cols*lb.scale)),sh=int(std::lround(source.rows*lb.scale));
  cv::Mat scaled,padded;cv::resize(source,scaled,cv::Size(sw,sh));
  cv::copyMakeBorder(scaled,padded,lb.pad_y,ih-sh-lb.pad_y,lb.pad_x,iw-sw-lb.pad_x,cv::BORDER_CONSTANT,cv::Scalar(114,114,114,255));
  size_t plane=size_t(iw)*ih;std::vector<float> input(plane*3);
  for(int y=0;y<ih;++y){auto row=padded.ptr<cv::Vec4b>(y);for(int x=0;x<iw;++x){size_t at=size_t(y)*iw+x;
    input[at]=row[x][2]/255.f;input[at+plane]=row[x][1]/255.f;input[at+2*plane]=row[x][0]/255.f;}}
  const auto& output=runner.infer(input.data());const auto& shape=runner.output_shape();
  if(shape.size()!=3||shape[0]!=1||shape[2]!=6)throw std::runtime_error("only end-to-end [1,N,6] output approved");
  auto boxes=zed_tracking::decode_end_to_end(output.data(),shape[1],.55);Json targets=Json::array();
  for(auto& b:boxes){
    if(b.class_id!=0||!std::isfinite(b.x1+b.x2+b.y1+b.y2+b.score))continue;
    double x1=b.x1,y1=b.y1,x2=b.x2,y2=b.y2;zed_tracking::map_box_back(lb,&x1,&y1,&x2,&y2);
    if(x2<=0||y2<=0||x1>=source.cols||y1>=source.rows)continue;
    targets.push_back({{"bbox_xyxy",{x1,y1,x2,y2}},{"confidence",b.score},{"detector_source","yolo"},
      {"track_id",0},{"mode","not_used"},{"reason","current_model_recomputed"}});
  }
  return targets;
}

int main(int argc,char** argv){
  try{
    if(argc!=4&&argc!=7)throw std::runtime_error("usage: extract_depth_reference SESSION OUTPUT STRIDE [ENGINE MODEL_ID FRAME_SELECTION_JSONL]");
    fs::path session=fs::canonical(argv[1]),output=fs::absolute(argv[2]);
    int stride=std::stoi(argv[3]);if(stride<1||stride>150)throw std::runtime_error("invalid stride");
    std::ifstream supervisor_stream("/tmp/zed_perception/supervisor.json");
    if(supervisor_stream){Json supervisor;supervisor_stream>>supervisor;
      if(supervisor.contains("pid")&&supervisor["pid"].is_number_integer()&&supervisor["pid"].get<int>()>0&&
         fs::exists(fs::path("/proc")/std::to_string(supervisor["pid"].get<int>())))
        throw std::runtime_error("stop live camera before offline replay");}
    std::unique_ptr<zed_tracking::TrtRunner> runner;std::set<std::pair<int,int>> selection;std::string evaluated_model;
    if(argc==7){
      evaluated_model=argv[5];if(evaluated_model.size()!=64||evaluated_model.find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::runtime_error("invalid model ID");
      if(engine_sha256(argv[4])!=evaluated_model)throw std::runtime_error("engine hash does not match model ID");
      std::ifstream list(argv[6]);std::string line;
      while(std::getline(list,line)){auto row=Json::parse(line);if(row.at("session_id")==session.filename().string())selection.emplace(row.at("segment_id").get<int>(),row.at("svo_position").get<int>());}
      if(selection.empty())throw std::runtime_error("no selected frames for this session");
      runner=std::make_unique<zed_tracking::TrtRunner>();runner->load(argv[4]);
    }
    if(fs::exists(output))throw std::runtime_error("output must be a new directory");
    std::ifstream observations(session/"observations.jsonl");if(!observations)throw std::runtime_error("no recorded observations");
    std::map<uint64_t,Json> rows;std::string text;
    while(std::getline(observations,text)){auto row=Json::parse(text);
      if(row.value("record_submitted",false)&&row.value("yolo_enabled",false)&&!row.at("targets").empty())
        rows.emplace(row.at("stamp_ns").get<uint64_t>(),std::move(row));}
    fs::create_directories(output/"images");std::ofstream samples(output/"samples.jsonl");
    std::ifstream manifest_stream(session/"manifest.json");Json manifest;manifest_stream>>manifest;
    Json report={{"source_session_id",session.filename().string()},{"stride",stride},{"reference","recomputed_stereo_not_external_ground_truth"},
      {"depth_mode","NEURAL_LIGHT"},{"depth_minimum_m",.3},{"depth_maximum_m",10.5},{"units","METER"},
      {"confidence_threshold",80},{"texture_confidence_threshold",100},
      {"prior_size_gate_used",false},{"recorded_model_id",manifest.at("model_id")},{"segments",Json::array()}};
    if(runner)report.update({{"evaluated_model_id",evaluated_model},{"input_nchw",runner->input_shape()},{"selected_frames",selection.size()}});
    int processed=0,matched=0,accepted=0;
    for(int segment=1;;++segment){
      auto source=session/"segments"/("segment_"+std::to_string(segment)+".svo2");if(!fs::exists(source))break;
      sl::Camera camera;sl::InitParameters init;init.input.setFromSVOFile(source.c_str());init.svo_real_time_mode=false;
      init.depth_mode=sl::DEPTH_MODE::NEURAL_LIGHT;init.coordinate_units=sl::UNIT::METER;
      init.depth_minimum_distance=.3f;init.depth_maximum_distance=10.5f;
      auto code=camera.open(init);if(code!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error(sl::toString(code).c_str());
      auto info=camera.getCameraInformation();const auto& config=info.camera_configuration;
      auto k=config.calibration_parameters.left_cam;
      sl::RuntimeParameters runtime;runtime.confidence_threshold=80;runtime.texture_confidence_threshold=100;
      sl::Mat depth,confidence,left;int count=camera.getSVONumberOfFrames();
      Json segment_report={{"segment",segment},{"frames",count},{"camera_serial",info.serial_number},{"sdk_version",sl::Camera::getSDKVersion().c_str()},
        {"fx",k.fx},{"fy",k.fy},{"cx",k.cx},{"cy",k.cy},{"width",config.resolution.width},{"height",config.resolution.height}};
      report["segments"].push_back(segment_report);
      int image_count=0;
      for(int position=0;position<count;position+=runner?1:stride){
        if(runner&&!selection.count({segment,position}))continue;
        camera.setSVOPosition(position);code=camera.grab(runtime);
        if(code!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("playback grab failed");
        ++processed;uint64_t stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns;
        auto found=rows.find(stamp);if(found==rows.end())continue;
        auto frame=found->second;
        if(frame.at("segment_id").get<int>()!=segment)throw std::runtime_error("timestamp segment mismatch");
        ++matched;
        auto original_k=frame.at("intrinsics");
        if(info.serial_number!=manifest.at("camera_serial").get<unsigned>() ||
          std::abs(k.fx-original_k.at("fx").get<double>())/k.fx>.01 ||
          std::abs(k.fy-original_k.at("fy").get<double>())/k.fy>.01 ||
          config.resolution.width!=frame.at("image_width").get<int>() || config.resolution.height!=frame.at("image_height").get<int>())
          throw std::runtime_error("playback calibration differs from recording");
        if(camera.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
           camera.retrieveMeasure(confidence,sl::MEASURE::CONFIDENCE,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS)
          throw std::runtime_error("metric depth retrieval failed");
        int w=int(config.resolution.width),h=int(config.resolution.height);
        if(runner){
          if(camera.retrieveImage(left,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("image retrieval failed");
          cv::Mat bgra(h,w,CV_8UC4,left.getPtr<sl::uchar1>(),left.getStepBytes());
          frame["targets"]=current_boxes(bgra,*runner);frame["model_id"]=evaluated_model;
        }
        std::vector<float> d(size_t(w)*h),c(d.size());
        for(int y=0;y<h;++y){
          std::copy_n(reinterpret_cast<const float*>(reinterpret_cast<const char*>(depth.getPtr<float>())+y*depth.getStepBytes()),w,d.begin()+size_t(y)*w);
          std::copy_n(reinterpret_cast<const float*>(reinterpret_cast<const char*>(confidence.getPtr<float>())+y*confidence.getStepBytes()),w,c.begin()+size_t(y)*w);
        }
        int index=0;
        for(const auto& target:frame.at("targets")){
          ++index;if(target.value("detector_source","")!="yolo")continue;
          auto box=target.at("bbox_xyxy").get<std::vector<double>>();if(box.size()!=4)continue;
          auto q=zed_tracking::assess_target_depth(d.data(),c.data(),w,h,box[0],box[1],box[2],box[3]);
          bool clipped=box[0]<=1||box[1]<=1||box[2]>=w-2||box[3]>=h-2;
          bool separated=q.accepted&&zed_tracking::target_separates_from_background(d.data(),c.data(),w,h,box[0],box[1],box[2],box[3],q.z);
          Json row={{"session_id",session.filename().string()},{"stamp_ns",stamp},{"sequence",frame.at("sequence")},{"segment_id",segment},
            {"svo_position",position},{"track_id",target.at("track_id")},{"bbox_xyxy",box},{"confidence",target.at("confidence")},
            {"model_id",frame.at("model_id")},{"camera_serial",info.serial_number},{"image_width",w},{"image_height",h},
            {"fx",k.fx},{"fy",k.fy},{"cx",k.cx},{"cy",k.cy},
            {"quality_accepted",q.accepted},{"clipped",clipped},{"background_separated",separated},{"depth_z_m",q.z},
            {"depth_valid_pixels",q.count},{"depth_valid_ratio",q.ratio},{"depth_iqr_m",q.iqr},{"reason",q.reason},
            {"online_mode",target.at("mode")},{"online_reason",target.at("reason")},{"prior_size_gate_used",false}};
          if(q.accepted&&!clipped&&separated){
            ++accepted;
            if(runner||image_count%3==0){
              if(camera.retrieveImage(left,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("image retrieval failed");
              cv::Mat bgra(h,w,CV_8UC4,left.getPtr<sl::uchar1>(),left.getStepBytes()),bgr,small;
              cv::cvtColor(bgra,bgr,cv::COLOR_BGRA2BGR);
              int l=std::clamp(int(box[0])-20,0,w-1),t=std::clamp(int(box[1])-20,0,h-1);
              int r=std::clamp(int(box[2])+21,l+1,w),b=std::clamp(int(box[3])+21,t+1,h);
              auto id=std::to_string(segment)+"_"+std::to_string(position)+"_"+std::to_string(index);
              cv::imwrite((output/"images"/(id+"_crop.jpg")).string(),bgr(cv::Rect(l,t,r-l,b-t)));
              cv::rectangle(bgr,cv::Point(int(box[0]),int(box[1])),cv::Point(int(box[2]),int(box[3])),cv::Scalar(0,255,0),2);
              cv::resize(bgr,small,cv::Size(640,360));cv::imwrite((output/"images"/(id+"_scene.jpg")).string(),small);
              row["image_id"]=id;
            }
            ++image_count;
          }
          samples<<row.dump()<<'\n';
        }
        if(processed%50==0){samples.flush();std::cout<<Json({{"segment",segment},{"position",position},{"processed",processed},{"matched",matched},{"independent_depth_candidates",accepted}}).dump()<<std::endl;}
      }
      camera.close();
    }
    if(report["segments"].empty())throw std::runtime_error("no SVO segments found");
    if(runner&&size_t(matched)!=selection.size())throw std::runtime_error("selected SVO frames did not all match recorded timestamps");
    report.update({{"processed",processed},{"matched",matched},{"independent_depth_candidates",accepted}});
    std::ofstream(output/"report.json")<<report.dump(2)<<'\n';std::cout<<report.dump()<<std::endl;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
