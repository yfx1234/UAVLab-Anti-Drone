#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace zed_tracking {
struct CameraMotionSample {
  bool valid{false};
  uint64_t timestamp_ns{0};
  std::array<double,4> quaternion_wxyz{1,0,0,0};
};
struct SkyIntrinsics {
  double fx{0},fy{0},cx{0},cy{0};
  int width{0},height{0};
};
inline bool normalized_quaternion(std::array<double,4>& q) {
  double norm=0;for(double v:q){if(!std::isfinite(v))return false;norm+=v*v;}
  if(norm<1e-12)return false;
  norm=std::sqrt(norm);for(double& v:q)v/=norm;return true;
}
inline std::array<double,4> quaternion_conjugate(std::array<double,4> q) {
  return {q[0],-q[1],-q[2],-q[3]};
}
inline std::array<double,4> quaternion_product(const std::array<double,4>& a,const std::array<double,4>& b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
    a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
    a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
    a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline std::array<double,3> rotate_ray(const std::array<double,4>& q,const std::array<double,3>& p) {
  const auto r=quaternion_product(quaternion_product(q,{0,p[0],p[1],p[2]}),quaternion_conjugate(q));
  return {r[1],r[2],r[3]};
}
inline bool motion_matches_image(CameraMotionSample& sample,double image_seconds) {
  if(!sample.valid || !std::isfinite(image_seconds) ||
     std::abs(sample.timestamp_ns*1e-9-image_seconds)>.025 ||
     !normalized_quaternion(sample.quaternion_wxyz))return false;
  return true;
}
} // namespace zed_tracking
