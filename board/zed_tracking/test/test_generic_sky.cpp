#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#if __has_include("zed_tracking/generic_sky_detector.hpp")
#include "zed_tracking/generic_sky_detector.hpp"
#include "zed_tracking/perception_quality.hpp"
using namespace zed_tracking;

static cv::Mat sky() { return cv::Mat(1080,1920,CV_8UC3,cv::Scalar(180,180,180)); }
static SkyCandidate box(float x, float y, float size=14) {
  return {x-size/2,y-size/2,x+size/2,y+size/2,.8f};
}
static CameraMotionSample motion(double seconds,double yaw=0) {
  return {true,static_cast<uint64_t>(seconds*1e9),{std::cos(yaw/2),0,std::sin(yaw/2),0}};
}
int main() {
  GenericSkyDetector detector;
  const auto options=sky_options("standard");
  assert(detector.detect(sky(),options).empty());
  cv::Mat gradient=sky();
  for(int y=0;y<gradient.rows;++y) gradient.row(y).setTo(cv::Scalar::all(130+y*80/1080));
  assert(detector.detect(gradient,options).empty());
  for(int value:{35,245}) {
    auto frame=sky(); cv::rectangle(frame,{950,530,14,12},cv::Scalar::all(value),cv::FILLED);
    auto candidates=detector.detect(frame,options);
    assert(!candidates.empty());
    assert(std::abs((candidates[0].x1+candidates[0].x2)/2-957)<4);
    assert(candidates[0].x2-candidates[0].x1>=12);
  }
  auto near=sky();
  cv::line(near,{865,445},{1055,635},cv::Scalar::all(35),14);
  cv::line(near,{865,635},{1055,445},cv::Scalar::all(35),14);
  auto large=detector.detect(near,options); assert(!large.empty());
  assert(large[0].x2-large[0].x1>175);
  SkyIntrinsics k{1266,1266,960,540,1920,1080};
  GenericSkyTracker hovering;
  assert(hovering.update({box(960,540)},k,1,motion(1),3).empty());
  assert(hovering.update({box(960,540)},k,1.067,motion(1.067),3).empty());
  auto held=hovering.update({box(960,540)},k,1.134,motion(1.134),3);
  assert(held.size()==1 && held[0].id==1);
  assert(!hovering.confirm_depth(1,true)); assert(!hovering.confirm_depth(1,true));
  assert(hovering.confirm_depth(1,true)); assert(!hovering.confirm_depth(1,false));
  assert(hovering.update({},k,1.201,motion(1.201),3).empty());
  GenericSkyTracker transient;
  transient.update({box(960,540)},k,1,motion(1),3);
  transient.update({},k,1.067,motion(1.067),3);
  assert(transient.update({box(960,540)},k,1.134,motion(1.134),3).empty());
  assert(transient.update({box(960,540)},k,1.201,motion(1.201),3).empty());
  GenericSkyTracker moving;
  for(int i=0;i<5;++i) {
    double seconds=2+i*.067,yaw=i*1.5*3.141592653589793/180;
    auto targets=moving.update({box(960-1266*std::tan(yaw),540)},k,seconds,motion(seconds,yaw),3);
    if(i>=2) assert(targets.size()==1 && targets[0].id==1 && targets[0].motion_compensated);
  }
  GenericSkyTracker fallback;
  for(int i=0;i<3;++i) {
    auto targets=fallback.update({box(960,540)},k,3+i*.067,CameraMotionSample{},3);
    if(i==2) assert(targets.size()==1 && !targets[0].motion_compensated);
  }
  GenericSkyTracker stale;
  for(int i=0;i<3;++i) {
    auto targets=stale.update({box(960,540)},k,4+i*.067,motion(1),3);
    if(i==2) assert(targets.size()==1 && !targets[0].motion_compensated);
  }
  auto crowded=sky();
  for(int y=40;y<1000;y+=70)for(int x=40;x<1850;x+=70)
    cv::rectangle(crowded,{x,y,12,12},cv::Scalar::all(35),cv::FILLED);
  auto many=detector.detect(crowded,options); assert(many.size()<=24);
  std::vector<SkyCandidate> adversarial;
  for(int i=0;i<100;++i) adversarial.push_back(box(20+i*18,540));
  GenericSkyTracker bounded;
  bounded.update(adversarial,k,5,motion(5),3);assert(bounded.track_count()<=64);
  assert(sky_options("high").confirmation_frames<options.confirmation_frames);
  bool invalid=false;try {sky_options("nonsense");}catch(const std::invalid_argument&){invalid=true;}
  assert(invalid);
  auto range=estimate_target_z(.30,1266,1266,13,10,false);
  assert(range.valid && range.z>29 && range.z<30);
  assert(!estimate_target_z(.30,1266,1266,13,10,true).valid);
  assert(!estimate_target_z(.30,1266,1266,7,5,false).valid);
  auto bearing=project_target(1100,540,0,1266,1266,960,540);
  assert(bearing.horizontal>0);
  auto bench=sky();cv::rectangle(bench,{950,530,14,12},cv::Scalar::all(35),cv::FILLED);
  detector.detect(bench,options);
  const auto start=std::chrono::steady_clock::now();
  for(int i=0;i<20;++i)detector.detect(bench,options);
  std::cout<<"generic sky behavioral tests passed; 1080p mean_ms="
           <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/20<<"\n";
}
#else
int main(){std::cerr<<"FAIL: generic sky detector is missing\n";return 1;}
#endif
