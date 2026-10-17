// depth_stats.hpp 单元测试:中值、置信度过滤、收缩、边界防御。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "zed_tracking/depth_stats.hpp"

namespace
{

constexpr int kWidth = 20;
constexpr int kHeight = 20;

// value_for(row, col) 返回深度值,NaN 表示无效。
std::vector<float> make_depth(float (* value_for)(int, int))
{
  std::vector<float> depth(kWidth * kHeight, std::nanf(""));
  for (int row = 0; row < kHeight; ++row) {
    for (int col = 0; col < kWidth; ++col) {
      depth[row * kWidth + col] = value_for(row, col);
    }
  }
  return depth;
}

}  // namespace

TEST(DepthStats, MedianIgnoresInvalidHalf)
{
  const auto depth = make_depth([](int row, int) {
      return row < 10 ? std::nanf("") : 5.0f;
    });
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    5, 5, 14, 14, /*shrink=*/1.0, /*min_valid=*/10,
    /*d_min=*/0.2, /*d_max=*/12.0, /*confidence_max=*/100.0);
  EXPECT_TRUE(sample.ok);
  EXPECT_NEAR(sample.median_m, 5.0, 1e-9);
  EXPECT_EQ(sample.valid_pixels, 50);
}

TEST(DepthStats, MedianHandlesMixedDistances)
{
  const auto depth = make_depth([](int row, int) {
      return row < 5 ? 4.0f : (row < 10 ? std::nanf("") : 6.0f);
    });
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    5, 5, 14, 14, 1.0, 10, 0.2, 12.0, 100.0);
  EXPECT_TRUE(sample.ok);
  // 框行 5..14:行 5..9 无效,行 10..14 为 6.0 -> 50 个有效,中值 6.0。
  EXPECT_NEAR(sample.median_m, 6.0, 1e-9);
  EXPECT_EQ(sample.valid_pixels, 50);
}

TEST(DepthStats, ConfidenceFiltersBadPixels)
{
  std::vector<float> depth = make_depth([](int, int) {return 5.0f;});
  std::vector<float> confidence(kWidth * kHeight, 10.0f);
  for (int col = 5; col <= 14; ++col) {
    confidence[7 * kWidth + col] = 99.0f;  // 整行置信度差
  }
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, confidence.data(), kWidth, kWidth, kHeight,
    5, 5, 14, 14, 1.0, 10, 0.2, 12.0, 95.0);
  EXPECT_TRUE(sample.ok);
  EXPECT_EQ(sample.valid_pixels, 90);
}

TEST(DepthStats, ShrinkReducesSampleArea)
{
  const auto depth = make_depth([](int, int) {return 5.0f;});
  const auto full = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    4, 4, 15, 15, 1.0, 1, 0.2, 12.0, 100.0);
  const auto shrunk = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    4, 4, 15, 15, 0.5, 1, 0.2, 12.0, 100.0);
  EXPECT_EQ(full.valid_pixels, 144);
  // 收缩 0.5 后边长约 6.x,总像素数必须少于满框。
  EXPECT_LT(shrunk.valid_pixels, 144);
  EXPECT_GE(shrunk.valid_pixels, 25);
  EXPECT_NEAR(shrunk.median_m, 5.0, 1e-9);
}

TEST(DepthStats, TooFewValidPixelsFails)
{
  const auto depth = make_depth([](int, int) {return std::nanf("");});
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    5, 5, 14, 14, 1.0, 12, 0.2, 12.0, 100.0);
  EXPECT_FALSE(sample.ok);
  EXPECT_EQ(sample.valid_pixels, 0);
}

TEST(DepthStats, OutOfRangeDepthsRejected)
{
  // d_max=10:框行 5..6 的 11 m 点被剔除,行 7..14 的 8 m 保留。
  const auto depth = make_depth([](int row, int) {return row < 7 ? 11.0f : 8.0f;});
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    5, 5, 14, 14, 1.0, 10, 0.2, 10.0, 100.0);
  EXPECT_TRUE(sample.ok);
  EXPECT_NEAR(sample.median_m, 8.0, 1e-9);
  EXPECT_EQ(sample.valid_pixels, 80);
}

TEST(DepthStats, BoxClampedToImageBounds)
{
  const auto depth = make_depth([](int, int) {return 5.0f;});
  // 框越界:不应崩溃,结果被夹到 [0, 19]。
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    -5, -5, 40, 40, 1.0, 1, 0.2, 12.0, 100.0);
  EXPECT_TRUE(sample.ok);
  EXPECT_EQ(sample.valid_pixels, kWidth * kHeight);
}

TEST(DepthStats, DegenerateBoxRejected)
{
  const auto depth = make_depth([](int, int) {return 5.0f;});
  const auto sample = zed_tracking::median_depth_in_box(
    depth.data(), kWidth, nullptr, 0, kWidth, kHeight,
    5, 5, 5, 5, 1.0, 1, 0.2, 12.0, 100.0);
  EXPECT_FALSE(sample.ok);
}
