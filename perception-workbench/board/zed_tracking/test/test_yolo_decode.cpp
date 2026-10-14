// yolo_decode.hpp 单元测试:端到端布局、原始布局 + NMS、自动判别。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "zed_tracking/yolo_decode.hpp"

TEST(YoloDecode, DetectsEndToEndLayout)
{
  EXPECT_TRUE(zed_tracking::looks_end_to_end({1, 300, 6}));
  EXPECT_TRUE(zed_tracking::looks_end_to_end({1, 4, 6}));
  EXPECT_FALSE(zed_tracking::looks_end_to_end({1, 6, 8400}));
  EXPECT_FALSE(zed_tracking::looks_end_to_end({1, 84, 8400}));
}

TEST(YoloDecode, EndToEndFiltersByScoreAndValidity)
{
  // 4 行:高分、次高分、低分、NaN 分。
  const std::vector<float> data = {
    10, 10, 50, 60, 0.9f, 0,
    20, 20, 80, 90, 0.8f, 0,
    30, 30, 40, 45, 0.2f, 0,
    1, 1, 2, 2, std::nanf(""), 0};
  const auto detections = zed_tracking::decode_end_to_end(data.data(), 4, 0.55);
  ASSERT_EQ(detections.size(), 2u);
  EXPECT_NEAR(detections[0].x1, 10.0f, 1e-6);
  EXPECT_NEAR(detections[0].score, 0.9f, 1e-6);
  EXPECT_NEAR(detections[1].y2, 90.0f, 1e-6);
}

TEST(YoloDecode, EndToEndRejectsDegenerateBoxes)
{
  const std::vector<float> data = {
    10, 10, 9, 60, 0.9f, 0,    // x2 < x1
    10, 10, 50, 8, 0.9f, 0};   // y2 < y1
  const auto detections = zed_tracking::decode_end_to_end(data.data(), 2, 0.55);
  EXPECT_TRUE(detections.empty());
}

TEST(YoloDecode, RawLayoutWithNmsSuppressesOverlap)
{
  // 1 类,属性 6(4 box + 1 score),3 个 anchor:
  // anchor0 (0,0,10,10) score .9;anchor1 (1,1,11,11) score .85 与前者高度重叠;
  // anchor2 (20,20,30,30) score .7 独立。
  std::vector<float> data(6 * 3, 0.0f);
  auto write = [&data](int anchor, const std::vector<float> & values) {
      const std::int64_t n = 3;
      data[0 * n + anchor] = values[0];
      data[1 * n + anchor] = values[1];
      data[2 * n + anchor] = values[2];
      data[3 * n + anchor] = values[3];
      data[4 * n + anchor] = values[4];
    };
  write(0, {0, 0, 10, 10, 0.9f});
  write(1, {1, 1, 11, 11, 0.85f});
  write(2, {20, 20, 30, 30, 0.7f});
  const auto detections = zed_tracking::decode_raw_with_nms(
    data.data(), 6, 3, /*class_count=*/1, 0.55, 0.5);
  ASSERT_EQ(detections.size(), 2u);
  EXPECT_NEAR(detections[0].x1, 0.0f, 1e-6);
  EXPECT_NEAR(detections[0].score, 0.9f, 1e-6);
  EXPECT_NEAR(detections[1].x1, 20.0f, 1e-6);
}

TEST(YoloDecode, RawLayoutSeparateClassesNotSuppressed)
{
  // 2 类、1 anchor 组两组同位置框:NMS 按类分别保留。
  const std::int64_t anchors = 2;
  std::vector<float> data((4 + 2) * anchors, 0.0f);
  data[0] = 0; data[anchors + 0] = 0; data[2 * anchors + 0] = 10;
  data[3 * anchors + 0] = 10;
  data[4 * anchors + 0] = 0.9f;  // 类 0 得分
  data[5 * anchors + 0] = 0.0f;  // 类 1 得分
  data[1] = 0; data[anchors + 1] = 0; data[2 * anchors + 1] = 10;
  data[3 * anchors + 1] = 10;
  data[4 * anchors + 1] = 0.0f;
  data[5 * anchors + 1] = 0.8f;  // 类 1 得分
  const auto detections = zed_tracking::decode_raw_with_nms(
    data.data(), 6, anchors, /*class_count=*/2, 0.55, 0.5);
  EXPECT_EQ(detections.size(), 2u);
}

TEST(YoloDecode, UnifiedEntryPicksLayout)
{
  const std::vector<float> e2e = {0, 0, 5, 5, 0.9f, 0};
  const auto from_e2e = zed_tracking::decode_detections(
    e2e.data(), {1, 1, 6}, 1, 0.55, 0.5);
  ASSERT_EQ(from_e2e.size(), 1u);
  EXPECT_NEAR(from_e2e[0].x2, 5.0f, 1e-6);
}
