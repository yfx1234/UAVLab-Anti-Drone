#ifndef NETGUN_INTERCEPT__NETGUN_FIRE_CONTROL_HPP_
#define NETGUN_INTERCEPT__NETGUN_FIRE_CONTROL_HPP_

#include <string>

namespace netgun_intercept
{

/// 择时火控配置（方案 v4 §3/§4）。
struct FireControlConfig
{
  // 主支：轴误差 ≤ primary_axis_error_rad → 立即开火（不等变差）。
  double primary_axis_error_rad = 0.017453292519943295;   // 1°
  // 兜底支：primary < 误差 ≤ window_axis_error_rad 持续 window_hold_s → 开火。
  double window_axis_error_rad = 0.03490658503988659;     // 2°（判窗）
  double window_hold_s = 0.2;
  // LOS 角速率门（沿用上游硬门语义）。
  double maximum_los_rate_rad_s = 0.5;
  // 距离门（与 NetgunModel 有效窗一致，由节点传入保持单一真源）。
  // 0.04s 去抖已按方案 v4 取消（原为 25fps 两帧保证，不再需要）。
  // 蒙射支（方案 v4 §4）：默认不启用，仅预留接口与参数。
  bool blind_shot_enabled = false;
  double blind_shot_max_axis_error_rad = 0.13962634015954636;  // 8°
  double blind_shot_exit_margin_s = 0.25;  // 距离窗即将穿出的前瞻时间
  // 距离门（与 NetgunModel 有效窗独立：火控可比裁判收缩留估计余量）
  double effective_range_min_m = 6.0;
  double effective_range_max_m = 10.0;
};

/// 单次更新输入；now_s 单调秒。
struct FireControlInput
{
  double now_s = 0.0;
  bool estimate_valid = false;
  double range_m = 0.0;
  double range_rate_mps = 0.0;            // 距离变化率（蒙射 A 支用）
  double axis_error_rad = 0.0;            // 机体光轴(射击基准轴)与预测射击方向夹角
  double los_rate_rad_s = 0.0;
  bool authorization_passed = false;      // 授权门（armed/offboard/高度/任务时间）
  bool engagement_timeout = false;        // 蒙射 B 支：任务超时未发射
};

enum class FireDecisionReason
{
  NONE = 0,
  PRIMARY_WINDOW,        // 主支：≤1° 即打
  WINDOW_HOLD,           // 兜底：2° 窗持续 0.2s
  BLIND_SHOT_EXIT,       // 蒙射 A：窗口即将穿出
  BLIND_SHOT_TIMEOUT,    // 蒙射 B：任务超时
  LATCHED,               // 单发已用
};

struct FireDecision
{
  bool fire = false;
  FireDecisionReason reason = FireDecisionReason::NONE;
  double window_hold_elapsed_s = 0.0;
  int shot_id = 0;
};

/// 择时火控（纯逻辑、无 ROS 依赖，可完全离线单测）。
/// 单发语义：触发一次后永久闭锁；无保持型门、无去抖。
class NetgunFireControl
{
public:
  explicit NetgunFireControl(const FireControlConfig & config = FireControlConfig());

  FireDecision update(const FireControlInput & input);

  bool latched() const;
  int shot_count() const;
  void reset();  // 仅供测试/任务重启使用；任务内单发闭锁不解除。

  const FireControlConfig & config() const;

  static std::string reason_name(FireDecisionReason reason);

private:
  bool distance_gate_passed(const FireControlInput & input) const;

  FireControlConfig config_;
  bool latched_ = false;
  int shot_count_ = 0;
  double window_hold_started_s_ = -1.0;
};

}  // namespace netgun_intercept

#endif  // NETGUN_INTERCEPT__NETGUN_FIRE_CONTROL_HPP_
