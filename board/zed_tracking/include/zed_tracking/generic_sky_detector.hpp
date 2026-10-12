#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "zed_tracking/camera_motion.hpp"

namespace zed_tracking {
struct GenericSkyOptions {
  int contrast_threshold{14};
  int confirmation_frames{3};
  size_t max_candidates{24};
};
inline GenericSkyOptions sky_options(const std::string& sensitivity) {
  if(sensitivity=="high")return {10,2,24};
  if(sensitivity=="standard")return {14,3,24};
  if(sensitivity=="low")return {22,4,24};
  throw std::invalid_argument("unknown generic sky sensitivity");
}
struct SkyCandidate {
  float x1{0},y1{0},x2{0},y2{0},score{0};
};
inline double sky_iou(const SkyCandidate& a,const SkyCandidate& b) {
  double intersection=std::max(0.f,std::min(a.x2,b.x2)-std::max(a.x1,b.x1))*
    std::max(0.f,std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
  double area=(a.x2-a.x1)*(a.y2-a.y1)+(b.x2-b.x1)*(b.y2-b.y1)-intersection;
  return area>0?intersection/area:0;
}

// Spatial contrast proposes unknown objects; the tracker supplies temporal support.
class GenericSkyDetector {
 public:
  std::vector<SkyCandidate> detect(const cv::Mat& image,const GenericSkyOptions& options) const {
    if(image.empty())return {};
    cv::Mat gray;
    if(image.channels()==4)cv::cvtColor(image,gray,cv::COLOR_BGRA2GRAY);
    else if(image.channels()==3)cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    else if(image.channels()==1 && image.depth()==CV_8U)gray=image;
    else throw std::invalid_argument("generic sky requires an 8-bit image");
    cv::Mat contrast=cv::Mat::zeros(gray.size(),CV_8U);
    add_contrast(gray,contrast,{9,25});
    cv::Mat half,wide=cv::Mat::zeros((gray.rows+1)/2,(gray.cols+1)/2,CV_8U),up;
    cv::resize(gray,half,wide.size(),0,0,cv::INTER_AREA);
    add_contrast(half,wide,{33,97});
    cv::resize(wide,up,gray.size(),0,0,cv::INTER_NEAREST);cv::max(contrast,up,contrast);
    cv::Mat mask;
    cv::threshold(contrast,mask,options.contrast_threshold,255,cv::THRESH_BINARY);
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
    cv::Mat labels,stats,centroids;
    int count=cv::connectedComponentsWithStats(mask,labels,stats,centroids,8,CV_32S);
    std::vector<SkyCandidate> candidates;
    for(int i=1;i<count;++i) {
      int x=stats.at<int>(i,cv::CC_STAT_LEFT),y=stats.at<int>(i,cv::CC_STAT_TOP);
      int w=stats.at<int>(i,cv::CC_STAT_WIDTH),h=stats.at<int>(i,cv::CC_STAT_HEIGHT);
      int area=stats.at<int>(i,cv::CC_STAT_AREA);
      if(area<4 || std::max(w,h)<4 || std::max(w,h)>500 ||
         std::max(w,h)>8*std::max(1,std::min(w,h)) || area>gray.total()*.06)continue;
      const cv::Rect rect(x,y,w,h);
      double score=cv::mean(contrast(rect),mask(rect))[0]/255.;
      candidates.push_back({float(x),float(y),float(x+w),float(y+h),float(std::min(1.,score))});
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.score>b.score;});
    if(candidates.size()>std::min(size_t(24),options.max_candidates))
      candidates.resize(std::min(size_t(24),options.max_candidates));
    return candidates;
  }
 private:
  static void add_contrast(const cv::Mat& gray,cv::Mat& contrast,std::initializer_list<int> scales) {
    cv::Mat light,dark;
    for(int scale:scales) {
      auto kernel=cv::getStructuringElement(cv::MORPH_RECT,{scale,scale});
      cv::morphologyEx(gray,dark,cv::MORPH_BLACKHAT,kernel);
      cv::morphologyEx(gray,light,cv::MORPH_TOPHAT,kernel);
      cv::max(contrast,dark,contrast);cv::max(contrast,light,contrast);
    }
  }
};

struct SkyTrack : SkyCandidate {
  uint32_t id{0};
  int consecutive_hits{0};
  bool motion_compensated{false};
};
class GenericSkyTracker {
 public:
  std::vector<SkyTrack> update(const std::vector<SkyCandidate>& candidates,const SkyIntrinsics& k,
      double seconds,CameraMotionSample motion,int confirmation_frames) {
    if(!std::isfinite(seconds)) {clear();return {};}
    const double dt=seconds-last_seconds_;
    if(last_seconds_>0 && (dt<=0 || dt>.5))clear();
    const bool current_valid=motion_matches_image(motion,seconds);
    bool compensated=current_valid && previous_motion_.valid && dt>0 && dt<=.25 && k.fx>0 && k.fy>0;
    std::array<double,4> relative{1,0,0,0};
    if(compensated) relative=quaternion_product(quaternion_conjugate(motion.quaternion_wxyz),previous_motion_.quaternion_wxyz);
    for(auto& item:tracks_) {
      item.used=false;
      item.track.motion_compensated=compensated && warp(item.track,k,relative);
    }
    std::vector<SkyTrack> output;
    for(size_t ci=0;ci<std::min(size_t(24),candidates.size());++ci) {
      const auto& candidate=candidates[ci];
      size_t best=tracks_.size();double best_cost=1e30;
      const double cx=(candidate.x1+candidate.x2)/2,cy=(candidate.y1+candidate.y2)/2;
      const double size=std::max(candidate.x2-candidate.x1,candidate.y2-candidate.y1);
      if(size<=0 || !std::isfinite(cx) || !std::isfinite(cy))continue;
      for(size_t ti=0;ti<tracks_.size();++ti) {
        const auto& item=tracks_[ti];if(item.used || seconds-item.last_seen>.3)continue;
        const auto& t=item.track;double old_size=std::max(t.x2-t.x1,t.y2-t.y1);
        if(old_size<=0 || size/old_size<.4 || size/old_size>2.5)continue;
        double distance=std::hypot(cx-(t.x1+t.x2)/2,cy-(t.y1+t.y2)/2);
        double gate=std::max(18.,std::max(size,old_size)*.9);
        if(distance>gate)continue;
        double cost=distance/gate+(1-sky_iou(candidate,t))*.25;
        if(cost<best_cost){best_cost=cost;best=ti;}
      }
      if(best==tracks_.size()) {
        if(tracks_.size()>=64)continue;
        Item item;static_cast<SkyCandidate&>(item.track)=candidate;item.track.id=next_id_++;
        item.track.consecutive_hits=1;item.last_seen=seconds;item.used=true;
        tracks_.push_back(item);best=tracks_.size()-1;
      } else {
        auto& item=tracks_[best];static_cast<SkyCandidate&>(item.track)=candidate;
        ++item.track.consecutive_hits;item.last_seen=seconds;item.used=true;
      }
      if(tracks_[best].track.consecutive_hits>=std::max(2,confirmation_frames))output.push_back(tracks_[best].track);
    }
    for(auto& item:tracks_)if(!item.used){item.track.consecutive_hits=0;item.depth_streak=0;}
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const auto& item){return seconds-item.last_seen>.3;}),tracks_.end());
    previous_motion_=motion;previous_motion_.valid=current_valid;last_seconds_=seconds;
    return output;
  }
  bool confirm_depth(uint32_t id,bool accepted) {
    for(auto& item:tracks_)if(item.track.id==id) {
      item.depth_streak=accepted?std::min(3,item.depth_streak+1):0;return item.depth_streak>=3;
    }
    return false;
  }
  void clear(){tracks_.clear();previous_motion_={};last_seconds_=0;}
  size_t track_count() const {return tracks_.size();}
 private:
  struct Item {SkyTrack track;double last_seen{0};int depth_streak{0};bool used{false};};
  static bool warp(SkyCandidate& box,const SkyIntrinsics& k,const std::array<double,4>& relative) {
    float x1=1e30f,y1=1e30f,x2=-1e30f,y2=-1e30f;
    for(float x:{box.x1,box.x2})for(float y:{box.y1,box.y2}) {
      auto p=rotate_ray(relative,{(x-k.cx)/k.fx,(y-k.cy)/k.fy,1});
      if(p[2]<=.1 || !std::isfinite(p[2]))return false;
      float u=float(k.fx*p[0]/p[2]+k.cx),v=float(k.fy*p[1]/p[2]+k.cy);
      x1=std::min(x1,u);y1=std::min(y1,v);x2=std::max(x2,u);y2=std::max(y2,v);
    }
    box.x1=x1;box.y1=y1;box.x2=x2;box.y2=y2;return true;
  }
  std::vector<Item> tracks_;
  CameraMotionSample previous_motion_;
  double last_seconds_{0};uint32_t next_id_{1};
};
} // namespace zed_tracking
