#include <sl/Camera.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>
namespace fs=std::filesystem;
int main(int argc,char** argv){
  try{
    if(argc!=5)throw std::runtime_error("usage: export_svo_rgb SVO output fps frames|verify|rgb_video|depth_video");
    std::string mode=argv[4];bool verify=mode=="verify",video=mode=="rgb_video"||mode=="depth_video",depth=mode=="depth_video";
    if(!verify&&!video&&mode!="frames")throw std::runtime_error("invalid export mode");
    double fps=std::stod(argv[3]);
    if(!std::isfinite(fps)||fps<0||fps>30||(!video&&fps==0))throw std::runtime_error("invalid fps");
    sl::Camera camera;sl::InitParameters p;p.input.setFromSVOFile(argv[1]);p.svo_real_time_mode=false;
    p.depth_mode=depth?sl::DEPTH_MODE::NEURAL_LIGHT:sl::DEPTH_MODE::NONE;
    p.depth_minimum_distance=.3f;p.depth_maximum_distance=10.5f;
    auto result=camera.open(p);if(result!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error(sl::toString(result).c_str());
    auto info=camera.getCameraInformation();double native_fps=info.camera_configuration.fps;
    if(video)fps=native_fps;
    if(!std::isfinite(fps)||fps<=0)throw std::runtime_error("missing native SVO FPS");
    int expected=camera.getSVONumberOfFrames(),frames=0,exported=0,video_frames=0,repeats=0;uint64_t previous=0,first=0,first_capture=0,last_capture=0;
    SHA256_CTX stamps;SHA256_Init(&stamps);
    fs::path out=fs::absolute(argv[2]);std::ofstream mapping;cv::VideoWriter writer;cv::Mat previous_video;
    fs::path video_path=out/(mode+".mp4"),video_temp=out/(mode+".tmp.mp4");
    if(!verify){fs::create_directories(out);if(!video){mapping.exceptions(std::ios::failbit|std::ios::badbit);mapping.open(out/"mapping.jsonl.tmp");}}
    sl::Mat left,right;sl::RuntimeParameters runtime;
    while(true){result=camera.grab(runtime);if(result==sl::ERROR_CODE::END_OF_SVOFILE_REACHED)break;
      if(result!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("SVO grab failed during playback");
      ++frames;auto stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns;
      if(frames==1)first_capture=stamp;
      if(frames>1&&stamp<=last_capture)throw std::runtime_error("nonmonotonic SVO timestamp");
      last_capture=stamp;
      auto stamp_text=std::to_string(stamp)+"\n";SHA256_Update(&stamps,stamp_text.data(),stamp_text.size());
      if(verify){
        if(camera.retrieveImage(left,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
           camera.retrieveImage(right,sl::VIEW::RIGHT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
           left.getWidth()==0||right.getWidth()==0)throw std::runtime_error("stereo image playback verification failed");
        continue;
      }
      if(!video&&previous&&stamp-previous<uint64_t(1e9/fps))continue;
      previous=stamp;
      if(fs::space(out).available<10ull*1024*1024*1024)throw std::runtime_error("export disk reserve below 10 GiB");
      if(camera.retrieveImage(left,depth?sl::VIEW::DEPTH:sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("image retrieval failed");
      cv::Mat image(left.getHeight(),left.getWidth(),CV_8UC4,left.getPtr<sl::uchar1>(),left.getStepBytes());
      if(video){
        cv::Mat bgr;cv::cvtColor(image,bgr,cv::COLOR_BGRA2BGR);
        if(depth)cv::putText(bgr,"Recomputed depth visualization | 0.3-10.5m | not metric depth data",{20,35},cv::FONT_HERSHEY_SIMPLEX,.7,{255,255,255},2);
        if(!writer.isOpened()){
          writer.open(video_temp.string(),cv::VideoWriter::fourcc('m','p','4','v'),fps,bgr.size());
          if(!writer.isOpened())throw std::runtime_error("MP4 writer unavailable");
          first=stamp;
        }
        auto slot=llround(double(stamp-first)*fps/1e9);
        if(slot>video_frames+llround(fps*120))throw std::runtime_error("unexpected timestamp gap exceeds120s");
        while(video_frames<slot){writer.write(previous_video);++video_frames;++repeats;}
        if(video_frames==slot){writer.write(bgr);++video_frames;}
        previous_video=bgr;continue;
      }
      fs::path file=out/("frame_"+std::to_string(camera.getSVOPosition())+".png");
      if(!cv::imwrite(file.string(),image))throw std::runtime_error("frame write failed");
      mapping<<nlohmann::json({{"svo_position",camera.getSVOPosition()},{"stamp_ns",stamp},{"file",file.string()}}).dump()<<'\n';++exported;
    }
    camera.close();
    if(frames<1||frames<expected)throw std::runtime_error("SVO frame count incomplete");
    if(!verify&&!video){mapping.close();fs::rename(out/"mapping.jsonl.tmp",out/"mapping.jsonl");}
    if(video){
      writer.release();cv::VideoCapture reader(video_temp.string());cv::Mat decoded;int count=0;
      while(reader.read(decoded)){
        if(decoded.cols!=int(info.camera_configuration.resolution.width)||decoded.rows!=int(info.camera_configuration.resolution.height))
          throw std::runtime_error("MP4 playback dimensions mismatch");
        ++count;
      }
      reader.release();
      if(count!=video_frames||count<1)throw std::runtime_error("MP4 playback frame count mismatch");
      fs::rename(video_temp,video_path);
    }
    unsigned char hash[SHA256_DIGEST_LENGTH];SHA256_Final(hash,&stamps);std::ostringstream digest;
    for(unsigned char c:hash)digest<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(c);
    nlohmann::json report={{"frames",frames},{"expected_frames",expected},{"exported_frames",exported},
      {"timestamp_sha256",digest.str()},{"mapping",(out/"mapping.jsonl").string()},
      {"capture_span_s",double(last_capture-first_capture)/1e9},
      {"unique_capture_hz",frames>1?double(frames-1)*1e9/double(last_capture-first_capture):0.0}};
    if(video){report.update({{"video",video_path.string()},{"video_frames",video_frames},{"video_fps",fps},
      {"video_width",info.camera_configuration.resolution.width},{"video_height",info.camera_configuration.resolution.height},
      {"video_duration_s",video_frames/fps},{"repeated_video_frames",repeats},{"video_timebase","native_capture_timestamps_with_gap_repeats"},
      {"depth_recomputed",depth},{"depth_mode",depth?"NEURAL_LIGHT":"NONE"},{"depth_visualization_only",depth}});}
    std::cout<<report.dump()<<std::endl;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
