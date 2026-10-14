// 深度统计纯函数:在检测框(可收缩)内取 ZED 深度图中值。
//
// ZED 深度图约定:float 米制,无效点为 NaN/Inf/<=0;
// 置信度图为 float 0-100,数值越低越可信(SDK 惯例与预览程序一致)。

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace zed_tracking
{

struct DepthSample
{
  bool ok{false};
  double median_m{0.0};
  double p25_m{0.0};
  double p75_m{0.0};
  int valid_pixels{0};
};

// 在 [x1,y1,x2,y2](全分辨率像素,含端点;会被夹到图像范围内)按
// shrink(0-1,向中心收缩边长比)采样深度,过滤:有限、(d_min, d_max]
// 开区间、置信度 <= confidence_max。
// 深度/置信度指针可为空(空则跳过对应过滤),步长以 float 元素计。
inline DepthSample median_depth_in_box(
  const float * depth, std::size_t depth_stride,
  const float * confidence, std::size_t confidence_stride,
  int image_width, int image_height,
  int x1, int y1, int x2, int y2,
  double shrink, int min_valid_pixels,
  double d_min, double d_max, double confidence_max)
{
  DepthSample sample;
  if (depth == nullptr || x2 <= x1 || y2 <= y1) {
    return sample;
  }
  x1 = std::max(0, x1);
  y1 = std::max(0, y1);
  x2 = std::min(image_width - 1, x2);
  y2 = std::min(image_height - 1, y2);
  if (x2 <= x1 || y2 <= y1) {
    return sample;
  }
  const double cx = 0.5 * (x1 + x2);
  const double cy = 0.5 * (y1 + y2);
  // shrink 语义:保留边长比(1.0=原框,0.5=边长减半)。
  const double keep = std::clamp(shrink, 0.05, 1.0);
  const int sx1 = static_cast<int>(std::floor(cx + (x1 - cx) * keep));
  const int sy1 = static_cast<int>(std::floor(cy + (y1 - cy) * keep));
  const int sx2 = static_cast<int>(std::ceil(cx + (x2 - cx) * keep));
  const int sy2 = static_cast<int>(std::ceil(cy + (y2 - cy) * keep));
  if (sx2 < x1 || sy2 < y1 || sx1 > x2 || sy1 > y2) {
    return sample;
  }
  const int px1 = std::max(sx1, x1), py1 = std::max(sy1, y1);
  const int px2 = std::min(sx2, x2), py2 = std::min(sy2, y2);
  if (px2 <= px1 || py2 <= py1) {
    return sample;
  }
  std::vector<float> values;
  values.reserve(static_cast<std::size_t>((px2 - px1 + 1) * (py2 - py1 + 1)));
  for (int row = py1; row <= py2; ++row) {
    const float * depth_row = depth + static_cast<std::size_t>(row) * depth_stride;
    const float * conf_row = (confidence != nullptr)
      ? confidence + static_cast<std::size_t>(row) * confidence_stride
      : nullptr;
    for (int col = px1; col <= px2; ++col) {
      const float value = depth_row[col];
      if (!std::isfinite(value) || value <= d_min || value > d_max) {
        continue;
      }
      if (conf_row != nullptr && !std::isfinite(conf_row[col])) {
        continue;
      }
      if (conf_row != nullptr && conf_row[col] > confidence_max) {
        continue;
      }
      values.push_back(value);
    }
  }
  sample.valid_pixels = static_cast<int>(values.size());
  if (sample.valid_pixels < min_valid_pixels) {
    return sample;
  }
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  sample.median_m = (n % 2 == 1)
    ? values[n / 2]
    : 0.5 * (values[n / 2 - 1] + values[n / 2]);
  sample.p25_m = values[n / 4];
  sample.p75_m = values[(3 * n) / 4];
  sample.ok = true;
  return sample;
}

}  // namespace zed_tracking
