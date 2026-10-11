#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <map>
#include "zed_tracking/yolo_decode.hpp"

namespace zed_tracking {
inline bool model_change_allowed(bool recording,const std::string& current,const std::string& proposed) {
  return !recording||current==proposed;
}
class SessionWorkCounter {
  std::mutex mutex_;std::condition_variable cv_;std::map<std::string,size_t> pending_;
public:
  void acquire(const std::string& id){if(id.empty())return;std::lock_guard<std::mutex> l(mutex_);++pending_[id];}
  void release(const std::string& id){if(id.empty())return;std::lock_guard<std::mutex> l(mutex_);auto it=pending_.find(id);
    if(it!=pending_.end()&&--it->second==0)pending_.erase(it);cv_.notify_all();}
  bool wait(const std::string& id,std::chrono::milliseconds timeout){std::unique_lock<std::mutex> l(mutex_);
    return cv_.wait_for(l,timeout,[&]{return pending_.count(id)==0;});}
};
struct CalibrationBinding {std::string model_id;unsigned int serial;int width,height;double fx,fy;};
inline bool calibration_matches(const CalibrationBinding& a,const CalibrationBinding& b) {
  return a.model_id==b.model_id&&a.serial==b.serial&&a.width==b.width&&a.height==b.height&&
    a.fx>0&&a.fy>0&&std::abs(a.fx-b.fx)/a.fx<.01&&std::abs(a.fy-b.fy)/a.fy<.01;
}
inline bool targets_are_fresh(bool camera_running,bool detector_running,double frame_age,double inference_age) {
  return camera_running&&detector_running&&frame_age>=0&&frame_age<1&&inference_age>=0&&inference_age<1;
}
inline uint64_t disk_reserve_bytes(double write_rate_bytes_s) {
  if(!std::isfinite(write_rate_bytes_s)||write_rate_bytes_s<0)write_rate_bytes_s=0;
  return std::max<uint64_t>(10ull*1024*1024*1024,uint64_t(std::ceil(write_rate_bytes_s*120)));
}
inline bool depth_matches_size_prior(double depth,double estimate) {
  // Coarse sanity gate only. Perspective/rotor extent still needs field calibration.
  return std::isfinite(depth+estimate)&&depth>0&&estimate>0&&depth>=estimate*.6&&depth<=estimate/ .6;
}
struct TargetDepthQuality {
  bool accepted{false}; double z{0},ratio{0},iqr{0}; int count{0};
  std::string reason{"no_depth"};
};
inline bool target_separates_from_background(const float* depth,const float* conf,int w,int h,
    double x1,double y1,double x2,double y2,double target_z) {
  if(!depth||target_z<=0)return false;
  double bw=x2-x1,bh=y2-y1;std::vector<float> ring;
  int l=std::max(0,int(x1-bw*.25)),r=std::min(w-1,int(x2+bw*.25));
  int t=std::max(0,int(y1-bh*.25)),b=std::min(h-1,int(y2+bh*.25));
  // Sampling the surrounding ring is independent of the box-center cluster.
  int step=std::max(1,int(std::max(bw,bh)/80));
  for(int y=t;y<=b;y+=step)for(int x=l;x<=r;x+=step){
    if(x>=x1&&x<=x2&&y>=y1&&y<=y2)continue;
    float z=depth[y*w+x];
    if(!std::isfinite(z)||z<=.3||z>10.5)continue;
    if(conf&&(!std::isfinite(conf[y*w+x])||conf[y*w+x]>80))continue;
    ring.push_back(z);
  }
  // Sky / distant background may have no measurable depth. That alone does
  // not invalidate a well-supported target cluster.
  if(ring.size()<12)return true;
  std::sort(ring.begin(),ring.end());double background=ring[ring.size()/2];
  return std::abs(background-target_z)/target_z>.05;
}
inline TargetDepthQuality assess_target_depth(const float* depth,const float* conf,
    int w,int h,double x1,double y1,double x2,double y2,double min_ratio=.10,
    double max_relative_iqr=.15,double confidence_max=80) {
  TargetDepthQuality q;
  if(!depth || w<=0 || h<=0 || !std::isfinite(x1+y1+x2+y2) || x2<=x1 || y2<=y1)return q;
  double cx=(x1+x2)/2,cy=(y1+y2)/2;
  int l=std::clamp(int(std::floor(cx-(x2-x1)*.3)),0,w-1);
  int r=std::clamp(int(std::ceil(cx+(x2-x1)*.3)),0,w-1);
  int t=std::clamp(int(std::floor(cy-(y2-y1)*.3)),0,h-1);
  int b=std::clamp(int(std::ceil(cy+(y2-y1)*.3)),0,h-1);
  if(r<=l||b<=t)return q;
  std::vector<float> values,center;
  for(int y=t;y<=b;++y)for(int x=l;x<=r;++x) {
    float z=depth[y*w+x];
    if(!std::isfinite(z)||z<=.3||z>10.5)continue;
    if(conf && (!std::isfinite(conf[y*w+x])||conf[y*w+x]>confidence_max))continue;
    values.push_back(z);
    if(std::abs(x-cx)<=(x2-x1)*.1 && std::abs(y-cy)<=(y2-y1)*.1)center.push_back(z);
  }
  q.count=int(values.size());q.ratio=double(q.count)/((r-l+1)*(b-t+1));
  if(q.count<12 || q.ratio<min_ratio){q.reason="sparse_depth";return q;}
  double center_area=std::max(1.0,(std::floor((x2-x1)*.2)+1)*(std::floor((y2-y1)*.2)+1));
  if(center.size()<3 || center.size()/center_area<min_ratio){q.reason="no_center_support";return q;}
  std::sort(center.begin(),center.end());
  double seed=center[center.size()/2];
  auto supports_seed=[&](float z){return std::abs(z-seed)/seed<=.10;};
  int center_support=int(std::count_if(center.begin(),center.end(),supports_seed));
  if(double(center_support)/center.size()<.60){q.reason="mixed_surfaces";return q;}
  // Resolve the depth cluster supported by the target center; the full box
  // median would otherwise select background around thin quadrotor arms.
  std::vector<float> cluster;
  std::copy_if(values.begin(),values.end(),std::back_inserter(cluster),supports_seed);
  q.count=int(cluster.size());q.ratio=double(q.count)/((r-l+1)*(b-t+1));
  if(q.count<12||q.ratio<min_ratio){q.reason="sparse_target_cluster";return q;}
  std::sort(cluster.begin(),cluster.end());q.z=cluster[cluster.size()/2];
  q.iqr=cluster[cluster.size()*3/4]-cluster[cluster.size()/4];
  if(q.iqr/q.z>max_relative_iqr){q.reason="mixed_surfaces";return q;}
  q.accepted=true;q.reason="heuristic_center_support";return q;
}
struct TargetProjection {double x,y,z,distance,horizontal,vertical;std::array<double,3> los;};
inline TargetProjection project_target(double u,double v,double z,double fx,double fy,double cx,double cy) {
  double a=(u-cx)/fx,b=(v-cy)/fy,n=std::sqrt(a*a+b*b+1);
  return {a*z,b*z,z,z*n,std::atan2(a,1),std::atan2(b,std::sqrt(1+a*a)),{a/n,b/n,1/n}};
}
struct EstimatedZ {bool valid;double z;};
inline EstimatedZ estimate_target_z(double size,double fx,double fy,double bw,double bh,bool clipped) {
  if(clipped||!std::isfinite(size+fx+fy+bw+bh)||size<=0||fx<=0||fy<=0||std::max(bw,bh)<8)return {false,0};
  // Which physical extent this projection corresponds to needs field calibration.
  double normalized_extent=std::max(bw/fx,bh/fy);
  return {normalized_extent>0,size/normalized_extent};
}
class TargetAssociation {
  struct Track {std::uint32_t id;Detection box;double seen;int streak;};
  std::vector<Track> tracks_;std::uint32_t next_{1};
public:
  void reset(){tracks_.clear();} // ID allocator remains monotonic within a run.
  std::vector<std::uint32_t> assign(const std::vector<Detection>& boxes,double now) {
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track& t){return now-t.seen>.5;}),tracks_.end());
    std::vector<std::uint32_t> ids;std::vector<std::uint32_t> used;
    for(const auto& box:boxes){
      Track* best=nullptr;double score=.2,second=0;
      for(auto& t:tracks_)if(t.box.class_id==box.class_id && std::find(used.begin(),used.end(),t.id)==used.end()) {
        double iou=box_iou(box,t.box);
        if(iou>score){second=score;score=iou;best=&t;}else second=std::max(second,iou);
      }
      if(best && score-second>.05){best->box=box;best->seen=now;ids.push_back(best->id);}
      else{tracks_.push_back({next_++,box,now,0});ids.push_back(tracks_.back().id);}
      used.push_back(ids.back());
    }
    return ids;
  }
  bool depth_ready(std::uint32_t id,bool accepted){for(auto& t:tracks_)if(t.id==id){t.streak=accepted?t.streak+1:0;return t.streak>=3;}return false;}
};
}
