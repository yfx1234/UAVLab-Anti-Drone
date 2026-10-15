#ifndef NETGUN_INTERCEPT__NETGUN_MODEL_HPP_
#define NETGUN_INTERCEPT__NETGUN_MODEL_HPP_

namespace netgun_intercept
{

/// 网枪简单模型参数（单一真源，V5-C 档；详见 docs/双目8m网枪拦截方案.md §2）。
/// 完整 V5 模型（RK4/时间基展开/Rician）后续可整体替换实现而不动调用方。
struct NetgunModelConfig
{
  double muzzle_speed_mps = 55.0;        // V5-C 初速
  double coverage_half_extent_m = 1.5;   // 3×3 m 覆盖面半幅（正方形）
  double deployment_time_s = 0.15;       // 展开时间（V5 72–147ms 取保守）
  double effective_range_min_m = 6.0;    // 任务口径有效下限
  double effective_range_max_m = 10.0;   // 任务口径有效上限（已确认取 10）
  int max_shots = 1;                     // 单发
};

/// 覆盖判定输入：目标相对射击轴的横向偏移（垂直于射轴平面内，米）与距离。
struct CoverageInput
{
  double lateral_x_m = 0.0;
  double lateral_y_m = 0.0;
  double range_m = 0.0;
};

class NetgunModel
{
public:
  explicit NetgunModel(const NetgunModelConfig & config = NetgunModelConfig());

  /// 直线匀速飞行时间（无动力学简化）；预留接口，V5 全模型可替换。
  double time_of_flight_s(double range_m) const;

  /// 触发提前量 = 飞行时间 + 展开时间（火控预测补偿用）。
  double trigger_lead_s(double range_m) const;

  /// 3×3 正方形覆盖面即时判定（裁判与火控共用的同一语义）。
  bool covered(const CoverageInput & input) const;

  bool range_within_effective(double range_m) const;

  const NetgunModelConfig & config() const;

private:
  NetgunModelConfig config_;
};

}  // namespace netgun_intercept

#endif  // NETGUN_INTERCEPT__NETGUN_MODEL_HPP_
