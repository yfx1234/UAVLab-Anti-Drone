#ifndef NETGUN_INTERCEPT__NETGUN_FIRE_CONTROL_HPP_
#define NETGUN_INTERCEPT__NETGUN_FIRE_CONTROL_HPP_

#include <array>
#include <string>

namespace netgun_intercept
{

/// 沿视线分解的测量 1σ（F-2：来源必须是 9×9 行主序的位置块）。
struct RangeLateralSigma
{
  double range_m = 0.0;
  double lateral_m = 0.0;
};

/// 从 9×9（行主序 81 元素，px/v/ax 分块）位置/速度/加速度协方差中，
/// 沿视线单位向量 u（NED，指向目标）取位置块分解距离/横向 1σ：
/// σ_range² = uᵀP_pos u；横向取 (trace(P_pos) − σ_range²)/2 的平方根。
/// F-2 修复后 strike 与本函数共用，下标 9*row+col 只此一处。
RangeLateralSigma sigma_range_lateral(
  const std::array<double, 81> & full_covariance, const double u[3]);

/// H-5（2026-10-19）：枪口位置偏置修正。瞄准点在机体参考点（相对估计
/// 原点）系下表达，弹丸从枪口出发——射击方向应由「瞄准点 − 枪口偏置
/// （NED）」给出。offset_ned 为机体系偏置经姿态旋转后的 NED 向量。
std::array<double, 3> apply_muzzle_offset(
  const std::array<double, 3> & aim_point_ned,
  const std::array<double, 3> & muzzle_offset_ned);

/// H-5：枪管轴在机体系的单位向量。机体攻击轴默认 = body -Z（与双目
/// 光轴同轴）；boresight_pitch/yaw 为枪管相对光轴的固定安装夹角
///（先绕 body-y 俯仰、再绕 body-z 偏航）。(0,0) 时与旧口径逐位一致。
std::array<double, 3> gun_axis_body(
  double boresight_pitch_rad, double boresight_yaw_rad);

/// I-3（2026-10-21）：枪管轴（NED）偏离竖直向上（NED -Z）的夹角，
/// 取值 [0, π]。零向量/非有限输入返回 NaN（调用方按"不通过"处理）。
/// 开火倾角门用：网弹不得朝近水平方向（人员/地面）射出。
double tilt_from_vertical_rad(const std::array<double, 3> & axis_ned);

/// I-3：手动兜底发射门控输入（节点侧逐拍汇总，判定为纯逻辑可离线单测）。
/// 默认口径（require_authorization=false）只保留物理安全门：飞控数据
/// 新鲜 + 已解锁 + 高度达标 + 倾角达标——不要求 offboard/稳定跟踪/
/// 有效估计，保证"自动一直不发"时操作手仍能人工发射（含遥控接管后）。
struct ManualFireGateInput
{
  bool manual_enabled = false;         // fire_mode 含 manual
  bool latched = false;                // 单发已用
  bool fcu_fresh = false;              // 姿态/状态/本地位置均在超时内
  bool armed = false;
  bool height_ok = false;              // 相对起飞原点高度 ≥ minimum_height_m
  bool tilt_ok = false;                // 枪管轴偏离竖直 ≤ fire_max_tilt_rad
  bool require_authorization = false;  // 严格口径：再要求武器授权+有效估计
  bool authorization_passed = false;
  bool estimate_valid = false;
};

/// 手动发射被拦下的首个原因（按判定顺序）；NONE = 可发射。
enum class ManualFireBlock
{
  NONE = 0,
  DISABLED,        // fire_mode=auto：按钮不生效
  LATCHED,         // 已发射（单发）
  FCU_STALE,       // 飞控姿态/状态/位置缺失或过期
  NOT_ARMED,       // 未解锁
  LOW_HEIGHT,      // 高度不足（或本地高度无效）
  TILT,            // 机体倾角过大
  NOT_AUTHORIZED,  // 严格口径下授权/估计未就绪
};

ManualFireBlock manual_fire_block(const ManualFireGateInput & input);
std::string manual_fire_block_name(ManualFireBlock block);

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
  bool measurement_quality_ok = true;     // R0-5：深度新鲜+距离/横向 σ 门
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
  MANUAL,                // I-1：面板/操作手手动发射（安全门控仍生效）
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

  // I-1（2026-10-19）：手动发射请求——绕过瞄准质量门（轴误差/判窗/
  // LOS/距离/σ，由操作手依据面板状态决定时机），但保留单发闭锁：
  // 首次调用即闭锁并返回 fire=true，此后返回 LATCHED。
  // 物理安全门（I-3：解锁/高度/倾角，严格口径再加授权+估计）不在本类
  // 内，由节点侧按 manual_fire_block() 把关后再调用。
  FireDecision request_manual();

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
