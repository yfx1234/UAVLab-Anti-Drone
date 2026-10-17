// YOLO 输出解析纯函数,支持两种布局并自动区分:
//
// 布局 A(端到端 NMS,本链路默认,export_yolo26_onnx.py --nms 输出):
//   形状 [1, 300, 6],每行 x1,y1,x2,y2,score,class(输入坐标系)。
// 布局 B(原始输出,ultralytics 导出未开 NMS):
//   形状 [1, 4+1+nc, N],属性为行、anchor 为列,需自行 NMS。
//
// 自动判定规则:最后一个维度 == 6 且倒数第二个维度 <= 1024 -> 布局 A;
// 否则按布局 B 解释(第二维等于 4+1+nc)。

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace zed_tracking
{

struct Detection
{
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};
  float score{0.0F};
  int class_id{0};
};

inline bool looks_end_to_end(const std::vector<std::int64_t> & dims)
{
  if (dims.size() < 2) {
    throw std::invalid_argument("detection tensor must have >=2 dims");
  }
  const std::int64_t last = dims.back();
  const std::int64_t second = dims[dims.size() - 2];
  return last == 6 && second > 0 && second <= 1024;
}

inline std::vector<Detection> decode_end_to_end(
  const float * data, std::int64_t rows, double score_threshold)
{
  std::vector<Detection> result;
  for (std::int64_t row = 0; row < rows; ++row) {
    const float * line = data + row * 6;
    const float score = line[4];
    if (!std::isfinite(score) || score < score_threshold) {
      continue;
    }
    Detection d;
    d.x1 = line[0];
    d.y1 = line[1];
    d.x2 = line[2];
    d.y2 = line[3];
    d.score = score;
    d.class_id = static_cast<int>(std::lround(line[5]));
    if (d.x2 > d.x1 && d.y2 > d.y1) {
      result.push_back(d);
    }
  }
  return result;
}

inline double box_iou(const Detection & a, const Detection & b)
{
  const double ix1 = std::max<double>(a.x1, b.x1);
  const double iy1 = std::max<double>(a.y1, b.y1);
  const double ix2 = std::min<double>(a.x2, b.x2);
  const double iy2 = std::min<double>(a.y2, b.y2);
  const double iw = std::max(0.0, ix2 - ix1);
  const double ih = std::max(0.0, iy2 - iy1);
  const double inter = iw * ih;
  const double area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
  const double area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
  const double union_area = area_a + area_b - inter;
  return union_area > 0.0 ? inter / union_area : 0.0;
}

// 布局 B:data 形如 [1, 4+1+nc, N](属性优先),本函数完成转置读取 + NMS。
inline std::vector<Detection> decode_raw_with_nms(
  const float * data, std::int64_t attribute_count, std::int64_t anchor_count,
  int class_count, double score_threshold, double iou_threshold)
{
  std::vector<Detection> candidates;
  const int class_offset = 4 + class_count;
  for (std::int64_t anchor = 0; anchor < anchor_count; ++anchor) {
    int best_class = -1;
    float best_score = 0.0F;
    for (int cls = 0; cls < class_count; ++cls) {
      const float score = data[static_cast<std::int64_t>(4 + cls) * anchor_count + anchor];
      if (std::isfinite(score) && score > best_score) {
        best_score = score;
        best_class = cls;
      }
    }
    if (best_class < 0 || best_score < score_threshold) {
      continue;
    }
    Detection d;
    d.x1 = data[anchor];
    d.y1 = data[anchor_count + anchor];
    d.x2 = data[2 * anchor_count + anchor];
    d.y2 = data[3 * anchor_count + anchor];
    d.score = best_score;
    d.class_id = best_class;
    if (d.x2 > d.x1 && d.y2 > d.y1) {
      candidates.push_back(d);
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Detection & a, const Detection & b) { return a.score > b.score; });
  std::vector<Detection> kept;
  for (const Detection & candidate : candidates) {
    bool suppressed = false;
    for (const Detection & survivor : kept) {
      if (survivor.class_id == candidate.class_id &&
        box_iou(survivor, candidate) > iou_threshold)
      {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(candidate);
    }
  }
  (void)class_offset;
  return kept;
}

// 统一入口:dims 为引擎输出张量形状(NCHW 或去掉 batch 的任意 >=2 维)。
inline std::vector<Detection> decode_detections(
  const float * data, const std::vector<std::int64_t> & dims,
  int class_count, double score_threshold, double iou_threshold)
{
  if (looks_end_to_end(dims)) {
    return decode_end_to_end(data, dims[dims.size() - 2], score_threshold);
  }
  if (dims.size() < 2) {
    throw std::invalid_argument("raw detection tensor must have >=2 dims");
  }
  return decode_raw_with_nms(
    data, dims[dims.size() - 2], dims.back(), class_count, score_threshold,
    iou_threshold);
}

}  // namespace zed_tracking
