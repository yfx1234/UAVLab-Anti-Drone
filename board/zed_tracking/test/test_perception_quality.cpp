#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <thread>
#include "zed_tracking/depth_stats.hpp"
#if __has_include("zed_tracking/perception_quality.hpp")
#include "zed_tracking/perception_quality.hpp"
#endif
int main() {
  constexpr int w=101,h=101;
  std::vector<float> d(w*h,std::numeric_limits<float>::quiet_NaN()),c(w*h,0);
#if __has_include("zed_tracking/perception_quality.hpp")
  auto assess=[&]() {return zed_tracking::assess_target_depth(d.data(),c.data(),w,h,0,0,100,100).accepted;};
#else
  auto assess=[&]() {return zed_tracking::median_depth_in_box(d.data(),w,c.data(),w,w,h,0,0,100,100,.6,12,.3,10.5,95).ok;};
#endif
  for(int x=20;x<32;++x) d[20*w+x]=4;
  if(assess()) {std::cerr<<"FAIL: sparse 12 pixels accepted as reliable target depth\n"; return 1;}
  std::fill(d.begin(),d.end(),8);
  for(int y=40;y<=60;++y) for(int x=40;x<=60;++x)d[y*w+x]=NAN;
  assert(!assess()); // surrounding background must not impersonate the target
  for(int y=0;y<h;++y) for(int x=0;x<w;++x)d[y*w+x]=x<50?4:9;
  assert(!assess()); // two depth surfaces
  std::fill(d.begin(),d.end(),4);
  assert(assess());
  std::fill(d.begin(),d.end(),8);
  for(int y=40;y<=60;++y)for(int x=40;x<=60;++x)d[y*w+x]=3;
  auto foreground=zed_tracking::assess_target_depth(d.data(),c.data(),w,h,0,0,100,100);
  assert(foreground.accepted && std::abs(foreground.z-3)<1e-8);
#if __has_include("zed_tracking/perception_quality.hpp")
  auto result=zed_tracking::project_target(50,50,10,100,100,0,50);
  assert(std::abs(result.distance-11.1803398875)<1e-8);
  assert(std::abs(result.x-5)<1e-8 && result.y==0);
  assert(std::abs(result.los[0]-.4472135955)<1e-8);
  auto small=zed_tracking::estimate_target_z(.3,100,100,7,5,false);
  assert(!small.valid);
  assert(!zed_tracking::estimate_target_z(.3,100,100,30,20,true).valid);
  assert(std::abs(zed_tracking::estimate_target_z(.3,100,100,30,20,false).z-1)<1e-9);
  { // Depth veto rules: missing or uncalibrated size estimate never rejects
    // a real measurement; only a calibrated prior with a real disagreement does.
    assert(!zed_tracking::depth_vetoed_by_size_prior(false,true,2.0,6.0));
    assert(!zed_tracking::depth_vetoed_by_size_prior(true,false,2.0,6.0));
    assert(zed_tracking::depth_vetoed_by_size_prior(true,true,2.0,6.0));
    assert(!zed_tracking::depth_vetoed_by_size_prior(true,true,3.6,6.0)); // boundary
    assert(!zed_tracking::depth_vetoed_by_size_prior(true,true,6.0,5.0));
  }
  assert(!zed_tracking::targets_are_fresh(false,true,.5,.4));
  assert(!zed_tracking::targets_are_fresh(true,false,.1,.1));
  assert(!zed_tracking::targets_are_fresh(true,true,.1,1.1));
  assert(zed_tracking::targets_are_fresh(true,true,.1,.1));
  assert(zed_tracking::frame_observations_fresh(true,.1));
  assert(!zed_tracking::frame_observations_fresh(false,.1));
  assert(!zed_tracking::frame_observations_fresh(true,1.0));
  assert(!zed_tracking::frame_observations_fresh(true,-.1));
  assert(!zed_tracking::frame_observations_fresh(true,NAN));
  assert(zed_tracking::disk_reserve_bytes(0)==10737418240ull);
  assert(zed_tracking::disk_reserve_bytes(100000000)==12000000000ull);
  assert(zed_tracking::depth_matches_size_prior(3,4.2));
  assert(!zed_tracking::depth_matches_size_prior(8,3));
  assert(!zed_tracking::depth_matches_size_prior(0,3));
  std::fill(d.begin(),d.end(),4);
  assert(!zed_tracking::target_separates_from_background(d.data(),c.data(),w,h,25,25,75,75,4));
  assert(zed_tracking::target_separates_from_background(d.data(),c.data(),w,h,25,25,75,75,2));
  zed_tracking::CalibrationBinding old{"model_a",42,1920,1080,1200,1200};
  assert(zed_tracking::calibration_matches(old,old));
  auto changed=old;changed.model_id="model_b";assert(!zed_tracking::calibration_matches(old,changed));
  changed=old;changed.serial=43;assert(!zed_tracking::calibration_matches(old,changed));
  zed_tracking::TargetAssociation tracks;
  zed_tracking::Detection box;box.x1=10;box.y1=10;box.x2=30;box.y2=30;box.class_id=0;
  auto first=tracks.assign({box},1)[0];tracks.reset();
  assert(tracks.assign({box},2)[0]>first);
  { // Fast small target: 17px box jumping 25px/frame must keep one ID
    // (2026-10-17 field video: pure same-frame IoU churned 41% of frames).
    zed_tracking::TargetAssociation fast;std::uint32_t prev=0;int held=0;
    for(int i=0;i<8;++i){
      zed_tracking::Detection b;b.class_id=0;
      b.x1=float(100+i*25);b.y1=200;b.x2=b.x1+17;b.y2=b.y1+17;b.score=.8f;
      auto id=fast.assign({b},i/15.0)[0];
      if(prev&&id==prev)++held;prev=id;
    }
    assert(held>=6);
  }
  { // Center-distance fallback: jump of ~1 box-diagonal (no IoU) still matches
    zed_tracking::TargetAssociation kept;
    zed_tracking::Detection a;a.class_id=0;a.x1=0;a.y1=0;a.x2=20;a.y2=20;a.score=.8f;
    auto id=kept.assign({a},0)[0];
    zed_tracking::Detection b=a;b.x1+=16;b.x2+=16;b.y1+=12;b.y2+=12;
    assert(kept.assign({b},1/15.0)[0]==id);
    zed_tracking::Detection c=a;c.x1+=60;c.x2+=60;c.y1+=45;c.y2+=45;
    assert(kept.assign({c},2/15.0)[0]!=id);
  }
  { // Ambiguity margin: same-frame pair never collapses onto one track
    zed_tracking::TargetAssociation ambiguous;
    zed_tracking::Detection a;a.class_id=0;a.x1=0;a.y1=0;a.x2=20;a.y2=20;a.score=.8f;
    zed_tracking::Detection twin=a;twin.x1=1;twin.x2=21;
    auto ids=ambiguous.assign({a,twin},0);
    assert(ids[0]!=ids[1]);
  }
  assert(!zed_tracking::model_change_allowed(true,"","new_model"));
  assert(zed_tracking::model_change_allowed(true,"same","same"));
  zed_tracking::SessionWorkCounter pending;
  pending.acquire("first");pending.acquire("first");
  assert(!pending.wait("first",std::chrono::milliseconds(5)));
  pending.release("first");assert(!pending.wait("first",std::chrono::milliseconds(5)));
  std::thread delayed([&]{std::this_thread::sleep_for(std::chrono::milliseconds(20));pending.release("first");});
  assert(pending.wait("first",std::chrono::milliseconds(100)));delayed.join();
#endif
  std::cout<<"PASS: sparse/background/mixed rejection and camera projection\n";
}
