#include <sl/Camera.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
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
    if(argc!=5)throw std::runtime_error("usage: export_svo_rgb SVO output fps frames|verify");
    double fps=std::stod(argv[3]);bool verify=std::string(argv[4])=="verify";
    if(!std::isfinite(fps)||fps<=0||fps>30)throw std::runtime_error("invalid fps");
    sl::Camera camera;sl::InitParameters p;p.input.setFromSVOFile(argv[1]);p.svo_real_time_mode=false;p.depth_mode=sl::DEPTH_MODE::NONE;
    auto result=camera.open(p);if(result!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error(sl::toString(result).c_str());
    int expected=camera.getSVONumberOfFrames(),frames=0,exported=0;uint64_t previous=0;
    SHA256_CTX stamps;SHA256_Init(&stamps);
    fs::path out=fs::absolute(argv[2]);std::ofstream mapping;
    if(!verify){fs::create_directories(out);mapping.exceptions(std::ios::failbit|std::ios::badbit);mapping.open(out/"mapping.jsonl.tmp");}
    sl::Mat left,right;sl::RuntimeParameters runtime;
    while(true){result=camera.grab(runtime);if(result==sl::ERROR_CODE::END_OF_SVOFILE_REACHED)break;
      if(result!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("SVO grab failed during playback");
      ++frames;auto stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns;
      auto stamp_text=std::to_string(stamp)+"\n";SHA256_Update(&stamps,stamp_text.data(),stamp_text.size());
      if(verify){
        if(camera.retrieveImage(left,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
           camera.retrieveImage(right,sl::VIEW::RIGHT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
           left.getWidth()==0||right.getWidth()==0)throw std::runtime_error("stereo image playback verification failed");
        continue;
      }
      if(previous&&stamp-previous<uint64_t(1e9/fps))continue;
      previous=stamp;
      if(fs::space(out).available<10ull*1024*1024*1024)throw std::runtime_error("export disk reserve below 10 GiB");
      if(camera.retrieveImage(left,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("RGB retrieval failed");
      cv::Mat image(left.getHeight(),left.getWidth(),CV_8UC4,left.getPtr<sl::uchar1>(),left.getStepBytes());
      fs::path file=out/("frame_"+std::to_string(camera.getSVOPosition())+".png");
      if(!cv::imwrite(file.string(),image))throw std::runtime_error("frame write failed");
      mapping<<nlohmann::json({{"svo_position",camera.getSVOPosition()},{"stamp_ns",stamp},{"file",file.string()}}).dump()<<'\n';++exported;
    }
    camera.close();
    if(frames<1||frames<expected)throw std::runtime_error("SVO frame count incomplete");
    if(!verify){mapping.close();fs::rename(out/"mapping.jsonl.tmp",out/"mapping.jsonl");}
    unsigned char hash[SHA256_DIGEST_LENGTH];SHA256_Final(hash,&stamps);std::ostringstream digest;
    for(unsigned char c:hash)digest<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(c);
    std::cout<<nlohmann::json({{"frames",frames},{"expected_frames",expected},{"exported_frames",exported},
      {"timestamp_sha256",digest.str()},{"mapping",(out/"mapping.jsonl").string()}}).dump()<<std::endl;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
