#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_offboard_tracking/msg/relative_estimate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "netgun_intercept/netgun_model.hpp"
#include <std_msgs/msg/string.hpp>
#include <virtual_interception_interfaces/msg/virtual_shot_event.hpp>

namespace netgun_intercept
{
namespace
{
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
enum class Phase { WAITING, TAKEOFF, TRACK, AIM, SETTLE, SPIN, DONE };
const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::WAITING: return "WAITING";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::TRACK: return "TRACK";
    case Phase::AIM: return "AIM";
    case Phase::SETTLE: return "SETTLE";
    case Phase::SPIN: return "SPIN";
    case Phase::DONE: return "DONE";
  }
  return "?";
}
}  // namespace

/// netgun 制导节点（架构 v2）：自动解锁+起飞+前馈跟踪。
class NetgunGuidanceNode : public rclcpp::Node
{
public:
  NetgunGuidanceNode() : Node("netgun_guidance_node")
  {
    takeoff_h_ = declare_parameter<double>("takeoff_height_m", 5.8);
    min_h_ = declare_parameter<double>("minimum_height_m", 1.6);
    gain_ = declare_parameter<double>("horizontal_position_gain", 1.2);
    max_spd_ = declare_parameter<double>("maximum_horizontal_speed_mps", 4.0);
    max_acc_ = declare_parameter<double>("maximum_horizontal_acceleration_mps2", 2.5);
    ff_ = declare_parameter<double>("feedforward_gain", 1.0);
    v_gain_ = declare_parameter<double>("altitude_position_gain", 1.0);
    max_vz_ = declare_parameter<double>("maximum_vertical_speed_mps", 1.5);
    engage_delay_ = declare_parameter<double>("engage_delay_s", 5.0);
    est_timeout_ = declare_parameter<double>("estimate_timeout_s", 0.5);
    ctrl_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    track_engage_speed_mps_ =
      declare_parameter<double>("track_engage_speed_mps", 0.8);
    prediction_horizon_s_ =
      declare_parameter<double>("prediction_horizon_s", 0.2);
    max_pred_offset_m_ =
      declare_parameter<double>("maximum_prediction_offset_m", 2.0);
    standoff_target_m_ =
      declare_parameter<double>("standoff_target_m", 7.5);
    accel_lead_s_ =
      declare_parameter<double>("acceleration_feedforward_lead_s", 0.0);
    integral_gain_ =
      declare_parameter<double>("horizontal_integral_gain", 0.3);
    yaw_servo_enabled_ = declare_parameter<bool>("yaw_servo_enabled", true);
    yaw_course_offset_rad_ =
      declare_parameter<double>("yaw_course_offset_rad", 0.0);
    yaw_max_rate_ = declare_parameter<double>("yaw_max_rate_rad_s", 3.0);
    yaw_engage_speed_mps_ =
      declare_parameter<double>("yaw_engage_speed_mps", 0.5);
    // BEARING_HOLD（2026-10-14 清单§八-3②）：估计失效后的主动再截获
    bearing_hold_enabled_ = declare_parameter<bool>("bearing_hold_enabled", true);
    approach_speed_mps_ =
      declare_parameter<double>("approach_speed_mps", 3.5);
    bearing_hold_sweep_rad_ =
      declare_parameter<double>("bearing_hold_sweep_rad", 0.44);
    bearing_hold_sweep_period_s_ =
      declare_parameter<double>("bearing_hold_sweep_period_s", 8.0);
    bearing_hold_recover_confirm_s_ =
      declare_parameter<double>("bearing_hold_recover_confirm_s", 0.3);
    // 减速感知交接（2026-10-15）：恢复须实际速度刹车到阈值下（或确认
    // 后超时兜底），防带 4m/s 动量交接在交差段过冲闪断
    bearing_hold_recover_speed_mps_ =
      declare_parameter<double>("bearing_hold_recover_speed_mps", 1.0);
    bearing_hold_recover_timeout_s_ =
      declare_parameter<double>("bearing_hold_recover_timeout_s", 2.0);
    bearing_hold_max_duration_s_ =
      declare_parameter<double>("bearing_hold_max_duration_s", 20.0);
    // 方位伺服（清单§八-3并入项）：瞄准线死区外的速度偏置，
    // 让机体在偏移收敛过程中提前倾斜对准（a≈g·xy/h 抵消几何轴误差）
    bearing_servo_enabled_ =
      declare_parameter<bool>("bearing_servo_enabled", true);
    bearing_servo_gain_ =
      declare_parameter<double>("bearing_servo_gain", 0.25);
    bearing_servo_deadband_m_ =
      declare_parameter<double>("bearing_servo_deadband_m", 0.15);
    // 摆动抑制（清单§八-4）：控制反馈用估计低通（0=关）
    est_lp_tau_ = declare_parameter<double>("estimate_lowpass_tau_s", 0.0);
    // 实机 offboard 门控（2026-10-14 用户定案）：rc_gated 模式下
    // 绝不发 VehicleCommand、未进 offboard 绝不发任何 setpoint、
    // 切出 offboard 立即停发 setpoint（仅保留 offboard_control_mode
    // 心跳——它是模式声明不是设定点，PX4 接受切换的前提）
    // 实机安全（R0-1，2026-10-17）：默认 rc_gated；engage_mode 只认
    // rc_gated/auto 两个值，auto 还需显式 allow_auto_engage——参数写错
    // （如 rc-gated）绝不能静默退化成"每 3s 给真飞控发解锁指令"。
    engage_mode_ = declare_parameter<std::string>("engage_mode", "rc_gated");
    allow_auto_engage_ =
      declare_parameter<bool>("allow_auto_engage", false);
    vehicle_command_target_system_ =
      declare_parameter<int>("vehicle_command_target_system", 2);
    if (engage_mode_ != "rc_gated" && engage_mode_ != "auto") {
      RCLCPP_FATAL(
        get_logger(), "engage_mode '%s' 不合法（只接受 rc_gated / auto）",
        engage_mode_.c_str());
      throw std::runtime_error("invalid engage_mode");
    }
    if (engage_mode_ == "auto" && !allow_auto_engage_) {
      RCLCPP_FATAL(
        get_logger(),
        "engage_mode=auto 需要显式 allow_auto_engage:=true（实机禁止）");
      throw std::runtime_error("auto engage not enabled");
    }
    // 飞控数据/武器授权新鲜度（R0-2）：断链停发心跳与指令，PX4 失效保护接管
    fcu_timeout_s_ = declare_parameter<double>("fcu_timeout_s", 0.5);
    weapon_auth_timeout_s_ =
      declare_parameter<double>("weapon_auth_timeout_s", 0.5);
    // R0-4：AIM 与火控统一瞄准口径——同一 muzzle/deployment、同一
    // aim_point（相对位置+相对速度×trigger_lead_s），PIP 用原始估计
    muzzle_speed_mps_ = declare_parameter<double>("muzzle_speed_mps", 55.0);
    deployment_time_s_ =
      declare_parameter<double>("deployment_time_s", 0.055);
    NetgunModelConfig aim_cfg;
    aim_cfg.muzzle_speed_mps = muzzle_speed_mps_;
    aim_cfg.deployment_time_s = deployment_time_s_;
    aim_model_ = NetgunModel(aim_cfg);
    // R1-2：目标速度坐标系（relative=旧口径默认；absolute 切换前需 SITL
    // 验证稳态间隙 ≥50% 改善）；R1-1：垂直加速度独立限幅；R2-4：机体
    // 横向加速度上限（0=关）
    target_velocity_frame_ =
      declare_parameter<std::string>("target_velocity_frame", "relative");
    yaw_course_bidirectional_ =
      declare_parameter<bool>("yaw_course_bidirectional", false);
    max_vertical_acc_ =
      declare_parameter<double>("max_vertical_acc_mps2", max_acc_);
    max_lateral_acc_ =
      declare_parameter<double>("max_lateral_acc_mps2", 0.0);
    // task origin 收敛后锁定（清单§八-5）：boot 期 EKF 未收敛值曾偏 6.6m
    task_origin_wait_converged_ =
      declare_parameter<bool>("task_origin_wait_converged", true);
    task_origin_settle_s_ =
      declare_parameter<double>("task_origin_settle_s", 2.0);
    task_origin_drift_max_m_ =
      declare_parameter<double>("task_origin_drift_max_m", 0.3);
    task_origin_timeout_s_ =
      declare_parameter<double>("task_origin_timeout_s", 20.0);
    // 预测拦截点导航（清单§八-8）：BEARING_HOLD 死区外推拦截点
    intercept_speed_mps_ =
      declare_parameter<double>("intercept_speed_mps", 8.0);
    bearing_hold_arrive_radius_m_ =
      declare_parameter<double>("bearing_hold_arrive_radius_m", 2.0);
    // 死区外推拦截开关（2026-10-15 矩阵证伪后默认关闭）：§八-8 外推点
    // 逼近在 headon@2-4/s_curve 上破坏再截获（外推点把飞机带向目标
    // 离去方向，追不出开火几何——40 轮矩阵 headon 6/10→2/10、s_curve
    // 8/10→5/10，且 headon@5 唯一命中轮未走 BH）。默认 false=回 §十七
    // matrix_geo2 验证口径（沿最后方位 approach_speed 逼近+扫描）。
    bearing_hold_intercept_nav_ =
      declare_parameter<bool>("bearing_hold_intercept_nav", false);
    // 主动瞄准 AIM（清单§八-0 完整版，2026-10-14 定案）：稳定跟踪+
    // 授权后主动倾斜机体令光轴指向目标，2° 门即打。B 路线=速度模式+
    // 加速度前馈（tanθ=a/g，借 MPC 姿态通道；推力/高度/漂移由 PX4
    // 闭环兜底）。火控 axis_err 用实测姿态独立验收真实指向，指令欠量
    // 只影响开门早晚不影响打的准。
    aim_mode_enabled_ = declare_parameter<bool>("aim_mode_enabled", true);
    aim_enter_delay_s_ = declare_parameter<double>("aim_enter_delay_s", 2.0);
    aim_max_tilt_rad_ = declare_parameter<double>("aim_max_tilt_rad", 0.35);
    aim_timeout_s_ = declare_parameter<double>("aim_timeout_s", 0.8);
    aim_max_target_speed_mps_ =
      declare_parameter<double>("aim_max_target_speed_mps", 100.0);
    aim_tilt_compensation_ =
      declare_parameter<double>("aim_tilt_compensation", 1.15);
    // 开火后 360° 原地旋转标志：人工核验"已激发"、提示飞手准备切出
    // offboard（位置+高度保持不破坏，仅 yaw 匀速推进）
    spin_flag_enabled_ = declare_parameter<bool>("spin_flag_enabled", true);
    spin_rate_rad_s_ = declare_parameter<double>("spin_rate_rad_s", 1.8);
    spin_total_rad_ = declare_parameter<double>("spin_total_rad", 6.2832);
    // SETTLE 先悬停再旋转（2026-10-15 用户定案流程）：开火→制动悬停→
    // 速度归零确认→起旋。开火动量前冲（v²/2a）消耗在本阶段，旋转严格
    // 发生在停稳后，"原地"性质干净。确认窗防速度过零误触发（回拉段
    // 峰值 ~0.5m/s 必须被排除）；超时兜底。
    spin_settle_speed_mps_ =
      declare_parameter<double>("spin_settle_speed_mps", 0.25);
    spin_settle_confirm_s_ =
      declare_parameter<double>("spin_settle_confirm_s", 0.3);
    spin_settle_timeout_s_ =
      declare_parameter<double>("spin_settle_timeout_s", 4.0);
    // 初始搜索慢自旋（2026-10-15）：从未捕获目标时原地慢转机头，
    // 把矩形视场扫成整圆。实机默认开；仿真 launch 显式关（矩阵口径）
    search_spin_enabled_ =
      declare_parameter<bool>("search_spin_enabled", true);
    search_spin_rate_rad_s_ =
      declare_parameter<double>("search_spin_rate_rad_s", 0.42);

    rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();

    est_sub_ = create_subscription<px4_offboard_tracking::msg::RelativeEstimate>(
      declare_parameter<std::string>("relative_estimate_topic", "/tracking/relative/estimate"),
      rclcpp::QoS(10),
      [this](px4_offboard_tracking::msg::RelativeEstimate::ConstSharedPtr m) { est_ = m; });
    lp_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      declare_parameter<std::string>("local_position_topic", "/px4_1/fmu/out/vehicle_local_position"),
      px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) {
        lp_rx_s_ = now().seconds();
        lp_valid_ = m->xy_valid && m->z_valid;
        lp_ = m;
      });
    st_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      declare_parameter<std::string>("status_topic", "/px4_1/fmu/out/vehicle_status"),
      px4_qos,
      [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m) {
        st_rx_s_ = now().seconds();
        st_ = m;
      });

    ob_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/px4_1/fmu/in/offboard_control_mode", 10);
    sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/px4_1/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/px4_1/fmu/in/vehicle_command", 10);
    to_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/tracking/task_origin_ned", rclcpp::QoS(1).transient_local());
    // 旧 simple_below_tracker 的隐性服务：就位信号（latched）。目标源
    // wait_for_pursuer_takeoff_ready=true 时等此信号才开始机动，否则目标
    // 会在追踪机爬升段提前离场（2026-10-12 定位）。
    hold_target_[2] = -takeoff_h_;
    // 开火事件：单发闩锁在火控侧，制导需被告知以脱离（否则开火后
    // 继续全程追击——实测飞出百米，实机上既浪费也危险）。
    // 开火后进 SPIN 旋转 360°（人工核验标志，位置保持不破坏），转满
    // 2π 后进 DONE 常规保持；spin 关闭时直接 DONE（旧行为）。
    shot_sub_ = create_subscription<virtual_interception_interfaces::msg::VirtualShotEvent>(
      declare_parameter<std::string>("shot_event_topic", "/tracking/virtual_shot_event"),
      rclcpp::QoS(10),
      [this](virtual_interception_interfaces::msg::VirtualShotEvent::ConstSharedPtr msg) {
        if ((phase_ == Phase::TRACK || phase_ == Phase::AIM) && !shot_seen_) {
          shot_seen_ = true;
          bearing_hold_ = false;
          hold_target_[0] = lp_ ? lp_->x : 0.0;
          hold_target_[1] = lp_ ? lp_->y : 0.0;
          hold_target_[2] = lp_ ? lp_->z : -takeoff_h_;
          if (spin_flag_enabled_) {
            phase_ = Phase::SETTLE;
            spin_accum_rad_ = 0.0;
            settle_ok_since_s_ = -1.0;
            shot_time_s_ = now().seconds();
            (void)frozen_yaw();  // 锚定当前航向（yaw 伺服关闭时也正确起步）
            RCLCPP_INFO(
              get_logger(),
              "shot event → SETTLE (brake to stop, then 360° spin)");
          } else {
            phase_ = Phase::DONE;
            RCLCPP_INFO(get_logger(), "shot event → phase DONE (position hold)");
          }
        }
      });
    // 武器授权（strike 单一真源，10Hz + transient_local）：AIM 进入
    // 条件。授权判据（就位锚+delay+稳定跟踪）在 strike 内部，guidance
    // 不重复计算一套（同 takeoff-ready 的模式）
    auth_sub_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>(
        "weapon_authorization_topic", "/tracking/weapon_authorization"),
      rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        auth_rx_s_ = now().seconds();
        weapon_auth_ = msg->data;
      });
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>(
        "takeoff_ready_topic", "/simulation/pursuer_takeoff_ready"),
      rclcpp::QoS(1).transient_local());

    // 调试状态话题（实物外场检索/失败归因，2026-10-15）：2Hz 独立定时器
    // 读成员状态拼 key=value 单行——echo/bag/grep 三友好，不碰控制流
    debug_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("debug_topic", "/debug/guidance_state"), 10);
    debug_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() { publish_debug(); });

    auto period = std::chrono::duration<double>(1.0 / ctrl_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period)),
      [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "guidance ready: takeoff=%.1f gain=%.2f ff=%.2f",
      takeoff_h_, gain_, ff_);
  }

private:
  // R0-2（2026-10-17）：飞控数据/武器授权新鲜度。数据断链（或 PX4 位置
  // 失效）时停发心跳与一切指令，让 PX4 按 offboard 丢失失效保护接管、
  // 飞手可立即夺回。注意这改变了旧语义"心跳永远在发"——心跳停滞现在是
  // "飞控数据断"的显式信号（面板/interceptor_status.py 已同步该语义）。
  bool fcu_inputs_fresh(double now_s) const
  {
    const bool lp_ok = lp_ && lp_valid_ && now_s - lp_rx_s_ <= fcu_timeout_s_;
    const bool st_ok = st_ && now_s - st_rx_s_ <= fcu_timeout_s_;
    return lp_ok && st_ok;
  }
  bool auth_effective(double now_s) const
  {
    return weapon_auth_ && now_s - auth_rx_s_ <= weapon_auth_timeout_s_;
  }
  // 交战状态复位（rc_gated 切出 / 飞控数据过期共用）。R0-3：覆盖全部
  // "第二次进入 offboard 会串场"的内部状态——保持点、斜率限幅种子、
  // 低通估计、积分、偏航锁定、授权/搜索计时、就位发布标志。
  void reset_engagement_state(const char* reason)
  {
    const bool was_active = phase_ != Phase::WAITING || bearing_hold_;
    phase_ = Phase::WAITING;
    hold_mode_ = true;
    bearing_hold_ = false;
    shot_seen_ = false;
    aim_stable_since_s_ = -1.0;
    aim_seeded_ = false;
    spin_accum_rad_ = 0.0;
    settle_ok_since_s_ = -1.0;
    ever_had_est_ = false;      // 重新搜索（自旋至再捕获）
    search_spin_logged_ = false;
    hold_target_[0] = lp_ ? lp_->x : 0.0;
    hold_target_[1] = lp_ ? lp_->y : 0.0;
    hold_target_[2] = lp_ ? lp_->z : -takeoff_h_;
    last_vx_ = last_vy_ = last_vz_ = 0.0;
    est_lp_[0] = est_lp_[1] = est_lp_[2] = 0.0;
    est_lp_seeded_ = false;
    integral_x_ = integral_y_ = 0.0;
    track_entry_s_ = -1.0;
    yaw_locked_ = false;
    yaw_above_since_s_ = -1.0;
    speed_above_since_s_ = -1.0;
    est_lost_since_s_ = -1.0;
    est_recover_since_s_ = -1.0;
    ready_published_ = false;
    if (was_active && now().seconds() - last_reset_log_s_ >= 1.0) {
      last_reset_log_s_ = now().seconds();
      RCLCPP_WARN(get_logger(), "engagement reset (%s) → standby", reason);
    }
  }
  // R0-3：切入 offboard 那一刻用当前位置/速度/航向初始化——不初始化时
  // 保持点滞留旧值（首飞锚点缺失或二次进入=上一次目标点），飞机会飞向
  // PX4 本地原点或旧目标；斜率限幅/低通从旧值起步会有指令阶跃。
  void on_engage_entry()
  {
    hold_target_[0] = lp_->x;
    hold_target_[1] = lp_->y;
    hold_target_[2] = -takeoff_h_;
    last_vx_ = lp_->vx;
    last_vy_ = lp_->vy;
    last_vz_ = lp_->vz;
    yaw_sp_ = lp_->heading;
    yaw_sp_init_ = true;
    est_lp_seeded_ = false;
    integral_x_ = integral_y_ = 0.0;
    hold_mode_ = true;
    bearing_hold_ = false;
    track_entry_s_ = -1.0;
  }
  void tick()
  {
    const double t_gate = now().seconds();
    if (!fcu_inputs_fresh(t_gate)) {
      reset_engagement_state("fcu data stale");
      return;
    }
    px4_msgs::msg::OffboardControlMode ob;
    ob.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    // R1-3：OCM 标志与实际 setpoint 类型一致——SETTLE/SPIN/DONE 发的
    // 也是位置设定点，position 标志必须同拍为 true（PX4 按标志决定
    // 需要哪类有效估计与接受哪类设定点）
    ob.position = (phase_ == Phase::TRACK && hold_mode_) ||
      phase_ == Phase::SETTLE || phase_ == Phase::SPIN ||
      phase_ == Phase::DONE;
    ob.velocity = true;
    ob.acceleration = (phase_ == Phase::AIM);  // AIM：加速度前馈通道（B 路线）
    ob_pub_->publish(ob);
    ++ob_sent_;

    // 心跳/指令计数 5s 节流行（2026-10-15）：PC 面板据此显示当前阶段、
    // 心跳连续性（ob_sent 增长）与指令流（sp_sent）。飞控数据过期时本行
    // 不执行（R0-2：心跳停发即"飞控数据断"信号）。
    const double alive_t = now().seconds();
    if (alive_t - last_alive_log_s_ >= 5.0) {
      last_alive_log_s_ = alive_t;
      RCLCPP_INFO(
        get_logger(), "guidance alive: ob=%llu sp=%llu phase=%s",
        static_cast<unsigned long long>(ob_sent_),
        static_cast<unsigned long long>(sp_sent_), phase_name(phase_));
    }

    if (!lp_ || !st_) return;
    const bool armed = st_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = st_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    const bool rc_gated = engage_mode_ == "rc_gated";
    if (!armed || !offboard) {
      if (rc_gated) {
        // 未获遥控器授权：零 setpoint 发布（用户实测 1.15.4 上非
        // offboard 态发 setpoint 会与遥控器竞争）。offboard_control_mode
        // 心跳照常（顶部已发），保证飞手拨杆时 PX4 能接受切换
        reset_engagement_state("offboard exited (RC authority)");
      } else {
        send_sp(0, 0, 0, 0.0);
      }
      const double t = now().seconds();
      if (start_s_ < 0) { start_s_ = t; }
      // task origin 收敛后锁定（清单§八-5，2026-10-14）：判据 =
      // 航向可用于控制 + 三轴速度静默（<0.15m/s）持续 settle 秒，
      // 且期间位置漂移 < drift 上限（防慢漂）；boot 后 timeout 秒
      // 兜底强制锁定（带 WARN）。engage 门控在锁定之后，防止起飞
      // 抢跑。task_origin_wait_converged=false 复现旧行为（立即锁）
      if (!to_pub_done_) {
        // 判据阈值 0.3：地面接触速度抖动会闪烁（首版 0.15 一拍不满足
        // 即重置探测窗，settle 永远凑不满——2026-10-14 冒烟实锤）
        const bool quality = task_origin_wait_converged_ ?
          (lp_->heading_good_for_control &&
          std::abs(lp_->vx) < 0.3 && std::abs(lp_->vy) < 0.3 &&
          std::abs(lp_->vz) < 0.3) : true;
        const bool timed_out = task_origin_wait_converged_ && start_s_ > 0.0 &&
          t - start_s_ > task_origin_timeout_s_;
        if (timed_out) {
          // 兜底必须在 quality 之外：判据永不满足时超时仍要强制锁定
          publish_task_origin(true);
        } else if (quality) {
          if (to_conv_since_s_ < 0.0) {
            to_conv_since_s_ = t;
            to_probe_[0] = lp_->x;
            to_probe_[1] = lp_->y;
          } else if (t - to_conv_since_s_ >= task_origin_settle_s_) {
            const bool stable =
              (std::abs(lp_->x - to_probe_[0]) + std::abs(lp_->y - to_probe_[1])) <
              task_origin_drift_max_m_;
            if (stable) {
              publish_task_origin(false);
            } else {
              to_conv_since_s_ = t;  // 漂移超限：重开探测窗
              to_probe_[0] = lp_->x;
              to_probe_[1] = lp_->y;
            }
          }
        } else {
          to_conv_since_s_ = -1.0;
        }
      }
      const bool engage_allowed =
        !rc_gated && (to_pub_done_ || !task_origin_wait_converged_);
      if (engage_allowed && start_s_ > 0.0 && t - start_s_ > engage_delay_) {
        // 周期重试（3s）直到 armed+offboard：一次性指令在 PX4 预检
        // 未就绪的 boot 会被拒绝，飞机永久趴地（2026-10-12 全几何矩阵
        // 定位：t_ready=-1 全程、range 恒为地面几何 12.17m）
        if (t - last_engage_s_ >= 3.0) {
          last_engage_s_ = t;
          send_cmd(176, 1.0f, 6.0f);
          send_cmd(400, 1.0f);
          ++engage_attempts_;
          RCLCPP_INFO(get_logger(), "auto-engage attempt %d", engage_attempts_);
        }
      }
      if (!to_pub_done_ && !task_origin_wait_converged_) {
        publish_task_origin(false);
      }
      return;
    }

    if (phase_ == Phase::WAITING) {
      on_engage_entry();
      phase_ = Phase::TAKEOFF;
      RCLCPP_INFO(get_logger(), "phase → TAKEOFF");
    }

    const double alt = -lp_->z;
    // NED z 向下为正：爬升 = 负 vz。垂直通道锚定 hold 段 trim 锁定的
    // z 设定点（hold_target_[2]）：速度模式若回固定高度 5.8m，会把
    // 锁好的站位间距当场拉掉 ~1.1m（2026-10-12 全几何矩阵实测
    // range 7.5→6.2 跳变后丢失目标）。
    const double vz_hold = clampd(
      v_gain_ * (hold_target_[2] - lp_->z), -max_vz_, max_vz_);

    if (phase_ == Phase::SETTLE) {
      // 开火后先悬停制动（位置保持照常、yaw 冻结），速度持续归零后
      // 才起旋。确认窗排除回拉段的过零点（实测回拉峰值 ~0.5m/s）；
      // 超时兜底直接起旋（慢一点的原地旋转仍远优于无标志）。
      send_pos_sp(hold_target_[0], hold_target_[1], hold_target_[2],
        0.0, 0.0, 0.0, frozen_yaw());
      const double spd = std::hypot(lp_->vx, lp_->vy);
      const double now_s7 = now().seconds();
      if (spd < spin_settle_speed_mps_) {
        if (settle_ok_since_s_ < 0.0) {settle_ok_since_s_ = now_s7;}
      } else {
        settle_ok_since_s_ = -1.0;
      }
      const bool settled =
        (settle_ok_since_s_ > 0.0 &&
          now_s7 - settle_ok_since_s_ >= spin_settle_confirm_s_) ||
        now_s7 - shot_time_s_ >= spin_settle_timeout_s_;
      if (settled) {
        phase_ = Phase::SPIN;
        spin_accum_rad_ = 0.0;
        RCLCPP_INFO(
          get_logger(),
          "settled (%.2f m/s after %.2fs) → SPIN (%.0f° in-place)",
          spd, now_s7 - shot_time_s_, spin_total_rad_ * 180.0 / M_PI);
      }
      return;
    }

    if (phase_ == Phase::SPIN) {
      // 360° 旋转标志：位置+高度保持照常（与 DONE 同款 setpoint），
      // 仅 yaw 匀速顺时针推进（NED yaw 正向=俯视顺时针），unwrap 累加。
      // 单发闩锁在火控侧，旋转不会引起二次开火；估计失效不影响
      // （位置保持锚定开火时刻位置）。
      const double dyaw = spin_rate_rad_s_ / ctrl_hz_;
      yaw_sp_ += dyaw;
      spin_accum_rad_ += dyaw;
      send_pos_sp(hold_target_[0], hold_target_[1], hold_target_[2],
        0.0, 0.0, 0.0, yaw_sp_);
      if (spin_accum_rad_ >= spin_total_rad_) {
        phase_ = Phase::DONE;
        RCLCPP_INFO(
          get_logger(), "spin done (%.0f°) → DONE position hold",
          spin_total_rad_ * 180.0 / M_PI);
      }
      return;
    }

    if (phase_ == Phase::DONE) {
      // 开火后原地保持（脱离追击）；航向锁定最后指令值
      send_pos_sp(hold_target_[0], hold_target_[1], hold_target_[2],
        0.0, 0.0, 0.0, frozen_yaw());
      return;
    }

    if (phase_ == Phase::TAKEOFF) {
      if (alt < takeoff_h_ - 0.3) {
        send_sp(0, 0, vz_hold, frozen_yaw());
        return;
      }
      phase_ = Phase::TRACK;
      track_entry_s_ = now().seconds();
      RCLCPP_INFO(get_logger(), "phase → TRACK (alt=%.1f)", alt);
      if (!ready_published_) {
        std_msgs::msg::Bool ready;
        ready.data = true;
        ready_pub_->publish(ready);
        ready_published_ = true;
        RCLCPP_INFO(get_logger(), "pursuer takeoff ready published");
      }
    }

    // 估计状态：只看 valid+时效（镜像语义）。不逐帧要求
    // measurement_accepted——瞬态期 IMM 拒帧会闪烁，造成模式抖动。
    bool est_ok = false;
    double tvx = 0.0, tvy = 0.0, tax = 0.0, tay = 0.0;
    if (est_ && est_->valid) {
      const double now_s = now().seconds();
      // R0-2：有符号年龄窗——时间戳在未来（时钟跳变）不再被当成新鲜数据
      const double est_age = now_s - est_->header.stamp.sec -
        est_->header.stamp.nanosec * 1e-9;
      est_ok = est_age >= -0.05 && est_age <= est_timeout_;
    }
    if (est_ok) {
      // R1-2：目标速度/加速度按参数选坐标系。absolute=相对量+本机量
      //（目标真实速度，稳态前馈有效）；relative=估计原始（默认旧口径）
      tvx = est_->velocity.x; tvy = est_->velocity.y;
      tax = est_->acceleration.x; tay = est_->acceleration.y;
      if (target_velocity_frame_ == "absolute" && lp_) {
        tvx += lp_->vx; tvy += lp_->vy;
        tax += lp_->ax; tay += lp_->ay;
      }
    }
    const double target_speed = est_ok ? std::hypot(tvx, tvy) : 0.0;

    // 首次捕获闩锁（搜索自旋停转判据）
    if (est_ok && !ever_had_est_) {
      ever_had_est_ = true;
      if (search_spin_logged_) {
        RCLCPP_INFO(
          get_logger(), "estimate acquired → search spin stopped");
      }
    }

    // 模式切换（单向）：目标静止 → 位置保持（消速度环悬停游荡，起跑前
    // 把水平间隙收敛到 cm 级）；估计速度越过阈值 → 永久切换速度+前馈
    // 跟踪。不做回退：IMM 收敛期速度估计噪声大（实测 0.15-5.8 m/s 波动），
    // 双向滞回会造成模式抖动（2026-10-12 探针实测）。
    const bool switch_allowed =
      hold_mode_ && track_entry_s_ > 0.0 && now().seconds() - track_entry_s_ >= 5.0;
    if (switch_allowed && target_speed > track_engage_speed_mps_) {
      // 持续 0.5s 确认：IMM 悬停速度噪声实测 ~0.3 m/s，TRACK 入口瞬间
      // 会冲过 0.3 阈值造成误切换（2026-10-12 探针实测，位置保持从未运行）
      const double now_s2 = now().seconds();
      if (speed_above_since_s_ < 0.0) {speed_above_since_s_ = now_s2;}
      if (now_s2 - speed_above_since_s_ >= 0.5) {
        hold_mode_ = false;
        integral_x_ = integral_y_ = 0.0;
        // 速度环种子 = 当前前馈（此时 IMM 已在运动目标上收敛；从零爬升
        // 会在起跑瞬态拉开 ~1m 间隙，甩出 ±2.5m 相机视场造成永久丢失）
        last_vx_ = clampd(ff_ * tvx, -max_spd_, max_spd_);
        last_vy_ = clampd(ff_ * tvy, -max_spd_, max_spd_);
        last_vz_ = 0.0;
        RCLCPP_INFO(
          get_logger(), "track → velocity mode (target %.2f m/s)", target_speed);
      }
    } else {
      speed_above_since_s_ = -1.0;
    }

    // 摆动抑制（清单§八-4）：控制反馈项用一阶低通估计（τ 参数，0=关）。
    // 只滤反馈（P/积分/保持点），前馈速度保持原始——摆动来自位置环
    // 对 IMM 噪声的响应（±0.23m@1.1s=1.8° 轴误差贴 2° 主门）
    if (est_ok) {
      if (!est_lp_seeded_) {
        // R0-3：第一条有效估计直接赋值——从 0 起低通会把反馈项拖 ~1s
        est_lp_[0] = est_->position.x;
        est_lp_[1] = est_->position.y;
        est_lp_[2] = est_->position.z;
        est_lp_seeded_ = true;
      } else {
        const double alpha = est_lp_tau_ > 0.0 ?
          std::min(1.0, (1.0 / ctrl_hz_) / est_lp_tau_) : 1.0;
        est_lp_[0] += alpha * (est_->position.x - est_lp_[0]);
        est_lp_[1] += alpha * (est_->position.y - est_lp_[1]);
        est_lp_[2] += alpha * (est_->position.z - est_lp_[2]);
      }
    } else {
      est_lp_seeded_ = false;
    }

    // AIM（B 路线）：速度匹配 + 加速度前馈主动倾斜。瞄准目标=PIP（est+
    // vel×tof，tof=|est|/55，与火控 PIP 同口径）；θ=atan2(|PIP 水平|,
    // |est_z|)×补偿系数、钳 aim_max_tilt；a=g·tanθ 朝 PIP 水平方向 →
    // 机体 -Z（光轴）倒向目标。yaw 保持当前值不动。
    // 【速度种子=进入时刻实际飞行速度并保持】（2026-10-15 smoke2 实测
    // 修正）：若按悬停（velocity=0）打，速度环反作用会在 ~1/K_v 秒内
    // 把前馈倾角指数吃掉（实测倾斜量≈0、轴误差不降）；匹配速度下
    // v_err≈0 无反作用，倾角保持到 2° 门打开，且偏移不增长（θ_req
    // 恒定，瞄准是收敛而非追逐）。种子取 lp_ 实际速度——hold 态进入
    // 时 last_v 无意义（hold 走位置设定点不更新它）。
    // 超时未开火或估计失效 → 回速度跟踪（重计稳定时间防抖；估计失效
    // 时火控互锁亦同步闭门）。
    if (phase_ == Phase::AIM) {
      const double now_s5 = now().seconds();
      if (!est_ok || now_s5 - aim_since_s_ > aim_timeout_s_) {
        phase_ = Phase::TRACK;
        aim_stable_since_s_ = -1.0;
        RCLCPP_INFO(
          get_logger(), "aim %s → resume velocity track",
          est_ok ? "timeout" : "est lost");
      } else {
        if (!aim_seeded_) {
          aim_vx_ = lp_->vx;
          aim_vy_ = lp_->vy;
          last_vx_ = aim_vx_; last_vy_ = aim_vy_; last_vz_ = vz_hold;
          aim_seeded_ = true;
          RCLCPP_INFO(
            get_logger(), "aim seed v=(%.2f,%.2f)", aim_vx_, aim_vy_);
        }
        // R0-4：PIP 用原始估计（非低通）+ 与火控同款 trigger_lead_s
        //（含展开时间），瞄准点口径与火控 aim_point 完全一致
        const double aim_range = std::sqrt(
          est_->position.x * est_->position.x +
          est_->position.y * est_->position.y +
          est_->position.z * est_->position.z);
        const double tof = aim_model_.trigger_lead_s(aim_range);
        const double pip_x = est_->position.x + est_->velocity.x * tof;
        const double pip_y = est_->position.y + est_->velocity.y * tof;
        const double pip_h = std::hypot(pip_x, pip_y);
        const double theta = std::min(
          std::atan2(pip_h, std::abs(est_->position.z)) * aim_tilt_compensation_,
          aim_max_tilt_rad_);
        const double a_mag = 9.80665 * std::tan(theta);
        double ax = 0.0, ay = 0.0;
        if (pip_h > 1e-6) {
          ax = a_mag * pip_x / pip_h;
          ay = a_mag * pip_y / pip_h;
        }
        double vz = vz_hold;
        if (alt < min_h_ && vz > 0) vz = 0;
        send_aim_sp(aim_vx_, aim_vy_, vz, ax, ay,
          frozen_yaw());
        return;
      }
    }

    // BEARING_HOLD 状态机（清单§八-3②，仅速度模式）：估计失效超
    // 时→记录最后方位，机头对准+慢扫描+朝方位逼近（复用保留参数
    // approach_speed_mps）；估计连续有效 recover_confirm 确认→回
    // 跟踪。逼近超 max_duration 停止只扫描（保底）。
    if (!hold_mode_) {
      const double now_s4 = now().seconds();
      if (est_ok) {
        est_lost_since_s_ = -1.0;
        // 恢复计时跨拍累积（2026-10-15 修复）：旧代码在此每拍重置
        // est_recover_since_s_ 导致 0.3s 确认永不满足、BH 一进不出。
        // 配套减速感知交接（见恢复处种子+逼近预减速），防交接振荡。
        if (std::hypot(est_lp_[0], est_lp_[1]) > 0.5) {
          bearing_hold_yaw_ = std::atan2(est_lp_[1], est_lp_[0]);
        }
        // 持续捕获目标绝对位姿（丢失时冻结为死区外推的起点）
        bh_abs_[0] = lp_->x + est_lp_[0];
        bh_abs_[1] = lp_->y + est_lp_[1];
        bh_vel_[0] = est_->velocity.x;
        bh_vel_[1] = est_->velocity.y;
        bh_time_ = now_s4;
        // 目标接近/远离判据（减速感知交接的几何门，2026-10-15）：
        // 相对速度·相对位置 <0=接近（headon 类：刹停在目标下方等
        // 穿越再交接）；>0=远离（追击/绕行类：确认后立即带种子交接
        // 恢复追赶——强刹会每循环丢进度，circle@4/5 实测闪断循环）
        bh_closing_ = (est_->velocity.x * est_lp_[0] +
          est_->velocity.y * est_lp_[1]) < 0.0;
        if (bearing_hold_) {
          if (est_recover_since_s_ < 0.0) { est_recover_since_s_ = now_s4; }
          // 减速感知交接（2026-10-15）：估计确认且实际速度已刹车到
          // 阈值以下才回跟踪（0.3s 确认窗结束时动量仍有 ~4m/s，直接
          // 交接会在交差段过冲闪断再进 BH——实测振荡根因）；确认后
          // 超时兜底强制交接，防速度降不下来时永久卡 BH。
          const bool est_confirmed =
            now_s4 - est_recover_since_s_ >= bearing_hold_recover_confirm_s_;
          const bool speed_settled =
            std::hypot(lp_->vx, lp_->vy) < bearing_hold_recover_speed_mps_ ||
            !bh_closing_;
          const bool recover_timeout =
            now_s4 - est_recover_since_s_ >= bearing_hold_recover_timeout_s_;
          if (est_confirmed && (speed_settled || recover_timeout)) {
            bearing_hold_ = false;
            // 速度种子=当前实际速度，残余动量经 send_sp 的 slew 平滑
            // 交还跟踪律
            last_vx_ = lp_->vx;
            last_vy_ = lp_->vy;
            RCLCPP_INFO(
              get_logger(),
              "bearing hold → resume tracking (seed v=%.2f,%.2f)",
              last_vx_, last_vy_);
          }
        }
      } else {
        est_recover_since_s_ = -1.0;
        if (est_lost_since_s_ < 0.0) { est_lost_since_s_ = now_s4; }
        if (!bearing_hold_ && bearing_hold_enabled_ &&
          now_s4 - est_lost_since_s_ >= est_timeout_)
        {
          bearing_hold_ = true;
          bearing_hold_since_s_ = now_s4;
          integral_x_ = integral_y_ = 0.0;
          RCLCPP_INFO(
            get_logger(),
            "est lost %.1fs → BEARING_HOLD %s (yaw→%.1f°, %.1f m/s)",
            est_timeout_, bearing_hold_intercept_nav_ ? "intercept" : "approach",
            bearing_hold_yaw_ * 180.0 / M_PI,
            bearing_hold_intercept_nav_ ? intercept_speed_mps_ : approach_speed_mps_);
        }
      }
    }

    // 航向伺服（交接清单 §八-1）：机头（=相机宽轴 H74°，实物安装
    // 2026-10-13 定案）对准目标航线，视场沿航线余量 ±3.0m→±5.65m。
    // 【必须 hold 模式即起转】（2026-10-13 A/B 首轮教训）：若等速度
    // 模式切换才转，起步瞬态（滞后 8-11°+倾斜 ~14°，同向叠加）在
    // 旋转完成前就把目标推出窄轴 ±22.5°。目标速度过阈 0.3s 确认后
    // 即转（IMM 低速方向噪声大，直接转会先转错方向）；速率 3 rad/s
    // 使 90° 旋转在偏移 <1m 时完成。BEARING_HOLD 期间改为对准最后
    // 方位+慢扫描；估计失效（非 BEARING_HOLD）或目标静止时锁定
    // 当前值；circle/s_curve 航线连续旋转，就最近短角差持续跟踪。
    double desired_yaw = yaw_sp_;
    bool yaw_has_target = false;
    if (bearing_hold_) {
      const double t_hold = now().seconds() - bearing_hold_since_s_;
      desired_yaw = bearing_hold_yaw_ + bearing_hold_sweep_rad_ *
        std::sin(2.0 * M_PI * t_hold / bearing_hold_sweep_period_s_);
      yaw_has_target = true;
    } else if (yaw_servo_enabled_ && est_ok &&
      target_speed > yaw_engage_speed_mps_)
    {
      const double now_s3 = now().seconds();
      if (yaw_above_since_s_ < 0.0) { yaw_above_since_s_ = now_s3; }
      if (now_s3 - yaw_above_since_s_ >= 0.3) {
        // R1-2：航向用目标速度坐标系；bidirectional 时宽视场轴前后
        // 对称——取当前航向更近的一侧，避免追过头时机头掉头 180°
        double course = std::atan2(tvy, tvx) + yaw_course_offset_rad_;
        if (yaw_course_bidirectional_) {
          double d = course - yaw_sp_;
          while (d > M_PI) { d -= 2.0 * M_PI; }
          while (d < -M_PI) { d += 2.0 * M_PI; }
          if (std::fabs(d) > M_PI / 2.0) { course += M_PI; }
        }
        desired_yaw = course;
        yaw_has_target = true;
      }
    } else {
      yaw_above_since_s_ = -1.0;
    }
    if (yaw_has_target) {
      while (desired_yaw - yaw_sp_ > M_PI) { desired_yaw -= 2.0 * M_PI; }
      while (desired_yaw - yaw_sp_ < -M_PI) { desired_yaw += 2.0 * M_PI; }
      const double max_dy = yaw_max_rate_ / ctrl_hz_;
      yaw_sp_ += clampd(desired_yaw - yaw_sp_, -max_dy, max_dy);
      if (!yaw_locked_) {
        yaw_locked_ = true;
        RCLCPP_INFO(
          get_logger(), "yaw servo engaged: %.1f° (target %.1f m/s)",
          yaw_sp_ * 180.0 / M_PI, target_speed);
      }
    }
    // 初始搜索慢自旋（2026-10-15，用户需求）：从未捕获目标时原地
    // 慢转机头——矩形视场（宽±37°沿机头/窄±22.5°垂直机头）下，
    // 天顶角 22.5°~37° 的环带天空只有机头转到对应方位才可见，自旋
    // 一周必然扫过（等效把捕获区从矩形扩成整圆）。首次估计有效即
    // 停（上方闩锁），交班 yaw 伺服/锁定；位置保持不受影响（yaw 与
    // 位置解耦，SPIN 段已验证同款手法）。
    const bool search_spinning = search_spin_enabled_ && !ever_had_est_ &&
      phase_ == Phase::TRACK;
    if (search_spinning) {
      if (!search_spin_logged_) {
        search_spin_logged_ = true;
        RCLCPP_INFO(
          get_logger(), "no estimate yet → search spin (%.0f°/s)",
          search_spin_rate_rad_s_ * 57.29577951308232);
      }
      yaw_sp_ += search_spin_rate_rad_s_ / ctrl_hz_;
    }
    const double yaw_cmd = frozen_yaw();

    // AIM 进入（完整版，TRACK 内）：稳定跟踪 aim_enter_delay + 武器授权
    // （strike 单一真源）+ 目标速度门。【两种跟踪形态都可进入】——1 m/s
    // 档稳态相对速度≈0，位置保持态（hold 分支先返回）本身就是稳定跟踪
    // （2026-10-15 冒烟实锤：进入判据放速度段会被 hold 分支挡住，轴误差
    // 恒卡 5-6°无法瞄准）。BEARING_HOLD/估计失效/授权未到重置稳定计时；
    // 本拍照常发跟踪指令，AIM 控制自下一拍起。
    if (phase_ == Phase::TRACK && aim_mode_enabled_ && !bearing_hold_) {
      const bool aim_ready = est_ok && auth_effective(now().seconds()) &&
        target_speed <= aim_max_target_speed_mps_;
      const double now_s6 = now().seconds();
      if (aim_ready) {
        if (aim_stable_since_s_ < 0.0) {aim_stable_since_s_ = now_s6;}
        if (now_s6 - aim_stable_since_s_ >= aim_enter_delay_s_) {
          phase_ = Phase::AIM;
          aim_since_s_ = now_s6;
          aim_seeded_ = false;
          RCLCPP_INFO(
            get_logger(),
            "stable track + weapon auth → AIM (tilt to target, max %.0f°)",
            aim_max_tilt_rad_ * 180.0 / M_PI);
        }
      } else {
        aim_stable_since_s_ = -1.0;
      }
    }

    if (hold_mode_) {
      // below-target 点 = 本机位置 + 低通估计；估计失效时锁定旧目标。
      if (est_ok) {
        hold_target_[0] = lp_->x + est_lp_[0];
        hold_target_[1] = lp_->y + est_lp_[1];
      }
      // 站位垂直闭环：task origin z 在 boot 期捕获的是 EKF 未收敛值
      //（±0.3-1m），直接平移站位距离——有的 boot 站位 5.2-5.9m，settle
      // 段 range 低于火控窗下限永远开不了火（2026-10-12 多轮矩阵 10/11
      // 失败轮的根因）。用相机估计（cm 级）把垂直间距锁到目标值。
      // NED z 向下为正：est_z=-5.9（间距 5.9）要拉大到 6.5 需【下降】
      // 0.6m（z_target = lp_z + (6.5 + est_z) = lp_z + 0.6）。
      // 首版符号写反导致追踪机抬头逼近目标（range 压到 4.1m）。
      hold_target_[2] = est_ok ?
        lp_->z + clampd(
        standoff_target_m_ + est_lp_[2], -1.0, 1.0) : -takeoff_h_;
      // 前馈清零：入轨初期 IMM 速度残值可达 ~4 m/s（爬升穿越不稳定区），
      // 注入位置环会把飞机拖出相机视场（±2.5m）→ 观测断流死锁。
      send_pos_sp(hold_target_[0], hold_target_[1], hold_target_[2],
        0.0, 0.0, 0.0, yaw_cmd);
      return;
    }

    // 速度+前馈跟踪（含 0.2s 预测，镜像 SimpleBelowTracker 同构，
    // 预测偏移上限 2m 防瞬态外推过冲）。反馈用低通估计，前馈保持原始。
    double vx = 0.0, vy = 0.0;
    if (bearing_hold_) {
      if (bearing_hold_intercept_nav_) {
        // 预测拦截点导航（清单§八-8，默认关）：目标绝对位姿按丢失时刻速度
        // 死区外推，朝预测点全速逼近（上限 intercept_speed_mps）；
        // 到达预测点 arrive 半径内则原地等目标进视场锥（迎敌穿越
        // 场景：等在航线下方）。超 max_duration 停止逼近只扫描
        // （防无限追陈旧外推）
        const double dt_loss = now().seconds() - bh_time_;
        const double tx = bh_abs_[0] + bh_vel_[0] * dt_loss;
        const double ty = bh_abs_[1] + bh_vel_[1] * dt_loss;
        const double dx = tx - lp_->x;
        const double dy = ty - lp_->y;
        const double dist = std::hypot(dx, dy);
        if (dist > bearing_hold_arrive_radius_m_) {
          bearing_hold_yaw_ = std::atan2(dy, dx);
        }
        if (dist > bearing_hold_arrive_radius_m_ &&
          now().seconds() - bearing_hold_since_s_ <= bearing_hold_max_duration_s_)
        {
          const double spd = std::min(intercept_speed_mps_, dist);
          vx = spd * dx / dist;
          vy = spd * dy / dist;
        }
      } else if (now().seconds() - bearing_hold_since_s_ <=
        bearing_hold_max_duration_s_)
      {
        // §十七 matrix_geo2 验证口径：沿最后有效方位慢速逼近
        // （approach_speed 3.5）+yaw 扫描（上方 yaw 段已有）。
        // 【看到即预减速】（仅来袭目标）：估计恢复有效起，逼近速度
        // 在确认窗内线性衰减到 0（经 send_sp slew 执行）——确认时刻
        // 飞机已近停，交接近无阶跃；目标远离（追击/绕行）不衰减，
        // 估计再丢则恢复全速逼近
        double decay = 1.0;
        if (est_recover_since_s_ > 0.0 && bh_closing_) {
          decay = std::max(0.0, 1.0 - (now().seconds() - est_recover_since_s_) /
            bearing_hold_recover_confirm_s_);
        }
        vx = approach_speed_mps_ * decay * std::cos(bearing_hold_yaw_);
        vy = approach_speed_mps_ * decay * std::sin(bearing_hold_yaw_);
      }
    } else if (est_ok) {
      double pred_x = est_lp_[0] + est_->velocity.x * prediction_horizon_s_;
      double pred_y = est_lp_[1] + est_->velocity.y * prediction_horizon_s_;
      const double pred_mag = std::hypot(pred_x, pred_y);
      if (pred_mag > max_pred_offset_m_) {
        pred_x *= max_pred_offset_m_ / pred_mag;
        pred_y *= max_pred_offset_m_ / pred_mag;
      }
      // 加速度前馈：起步/机动瞬态 IMM 速度滞后 ~0.4s（贡献 ~2m 间隙），
      // 用 IMM 加速度估计超前补偿（2026-10-12 几何探针定位）。
      // R1-2：前馈速度按 target_velocity_frame 选坐标系
      vx = gain_ * pred_x + ff_ * (tvx + tax * accel_lead_s_);
      vy = gain_ * pred_y + ff_ * (tvy + tay * accel_lead_s_);
      // 积分项：P+FF 无积分时稳态间隙 = v×(1/gain−horizon) ≈ 1-1.5m
      //（追踪机 lag 在目标后方，持续 5-8° 轴误差，2° 主门永远不开）。
      // 积分把间隙压回 0（正下方），kI=0.3、钳 ±0.8m/s 防风扰饱和。
      //（2026-10-14 摆动抑制：积分输入改低通估计，launch kI 0.3→0.1）
      integral_x_ = clampd(
        integral_x_ + est_lp_[0] * integral_gain_ / ctrl_hz_, -0.8, 0.8);
      integral_y_ = clampd(
        integral_y_ + est_lp_[1] * integral_gain_ / ctrl_hz_, -0.8, 0.8);
      vx += integral_x_;
      vy += integral_y_;
      // 方位伺服（清单§八-3 并入项）：瞄准线水平偏移死区外，加朝
      // 目标方向的速度偏置。速度环 P≈5 下 k=0.25 → a≈1.25·xy ≈
      // g·xy/h（h≈7.5）——偏移收敛过程中机体提前倾斜对准瞄准线，
      // 几何轴误差被倾斜抵消，对准时刻提前；死区内不动作（与摆动
      // 抑制不冲突）
      if (bearing_servo_enabled_) {
        const double aim_r = std::hypot(est_lp_[0], est_lp_[1]);
        if (aim_r > bearing_servo_deadband_m_) {
          const double bias = std::min(
            bearing_servo_gain_ * (aim_r - bearing_servo_deadband_m_), 2.0);
          vx += bias * est_lp_[0] / aim_r;
          vy += bias * est_lp_[1] / aim_r;
        }
      }
    }
    // 估计失效：指令经加速度限幅自然衰减到 0（send_sp 的 slew 限幅
    // 每拍向 0 收敛）——停住等再截获，绝不追 IMM 外推值飞（外推只在
    // ~0.5s 内可信，追它会越飞越偏，2026-10-12 几何探针定位）
    double vz = vz_hold;
    if (alt < min_h_ && vz > 0) vz = 0;  // NED 正=下降；低于下限禁止下降
    limit_horizontal_speed(vx, vy, max_spd_);  // R1-1：按矢量模限幅
    send_sp(vx, vy, vz, yaw_cmd);
  }

  void send_sp(double vx, double vy, double vz, double yaw)
  {
    // R1-1：水平斜率按矢量模限幅（斜向加速度不超 max_acc），垂直独立
    const double max_dv = max_acc_ / ctrl_hz_;
    const double max_dvz = max_vertical_acc_ / ctrl_hz_;
    double dvx = vx - last_vx_, dvy = vy - last_vy_;
    limit_horizontal_speed(dvx, dvy, max_dv);
    if (max_lateral_acc_ > 0.0) {
      // R2-4（可选）：机体横向（窄视场轴）加速度分量限幅，默认关闭
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double max_lat = max_lateral_acc_ / ctrl_hz_;
      const double fwd = c * dvx + s * dvy;
      double lat = -s * dvx + c * dvy;
      lat = clampd(lat, -max_lat, max_lat);
      dvx = c * fwd - s * lat;
      dvy = s * fwd + c * lat;
    }
    vx = last_vx_ + dvx;
    vy = last_vy_ + dvy;
    vz = last_vz_ + clampd(vz - last_vz_, -max_dvz, max_dvz);
    last_vx_ = vx; last_vy_ = vy; last_vz_ = vz;
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    // 未使用的设定点字段必须显式置 NaN（与 simple_below_tracker 同款）：
    // 默认 0 会被 PX4 当成"回原点"的位置/加速度设定点，压制速度指令，
    // 追踪机无法爬升（2026-10-12 联仿定位）。
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sp.position = {nan, nan, nan};
    sp.velocity[0] = vx; sp.velocity[1] = vy; sp.velocity[2] = vz;
    sp.acceleration = {nan, nan, nan};
    sp.jerk = {nan, nan, nan};
    sp.yaw = static_cast<float>(yaw);
    sp_pub_->publish(sp);
    ++sp_sent_;
  }

  void send_aim_sp(
    double vx, double vy, double vz, double ax, double ay, double yaw)
  {
    // AIM 专用：velocity + acceleration 前馈双通道（offboard_control_mode
    // 两位同开）。速度 slew 与 send_sp 同款——进入/退出 AIM 的指令斜坡
    // 自然衔接；加速度不经 slew（θ 指令本身已钳制在 aim_max_tilt 内）。
    const double max_dv = max_acc_ / ctrl_hz_;
    const double max_dvz = max_vertical_acc_ / ctrl_hz_;
    double dvx = vx - last_vx_, dvy = vy - last_vy_;
    limit_horizontal_speed(dvx, dvy, max_dv);
    if (max_lateral_acc_ > 0.0) {
      // R2-4（可选）：机体横向（窄视场轴）加速度分量限幅，默认关闭
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double max_lat = max_lateral_acc_ / ctrl_hz_;
      const double fwd = c * dvx + s * dvy;
      double lat = -s * dvx + c * dvy;
      lat = clampd(lat, -max_lat, max_lat);
      dvx = c * fwd - s * lat;
      dvy = s * fwd + c * lat;
    }
    vx = last_vx_ + dvx;
    vy = last_vy_ + dvy;
    vz = last_vz_ + clampd(vz - last_vz_, -max_dvz, max_dvz);
    last_vx_ = vx; last_vy_ = vy; last_vz_ = vz;
    dbg_ax_ = ax; dbg_ay_ = ay;  // 调试话题用（AIM 倾角指令）
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sp.position = {nan, nan, nan};
    sp.velocity[0] = vx; sp.velocity[1] = vy; sp.velocity[2] = vz;
    // z 加速度=0（无垂直前馈；垂直环照旧由速度通道闭环）
    sp.acceleration[0] = ax; sp.acceleration[1] = ay; sp.acceleration[2] = 0.0;
    sp.jerk = {nan, nan, nan};
    sp.yaw = static_cast<float>(yaw);
    sp_pub_->publish(sp);
    ++sp_sent_;
  }

  void send_pos_sp(
    double x, double y, double z, double vxff, double vyff, double vz,
    double yaw)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    sp.position = {x, y, z};
    sp.velocity = {
      static_cast<float>(vxff), static_cast<float>(vyff), static_cast<float>(vz)};
    sp.acceleration = {nan, nan, nan};
    sp.jerk = {nan, nan, nan};
    sp.yaw = static_cast<float>(yaw);
    sp_pub_->publish(sp);
    ++sp_sent_;
  }

  void send_cmd(double cmd, float p1, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand c;
    c.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    c.param1 = p1; c.param2 = p2; c.command = cmd;
    c.target_system = vehicle_command_target_system_; c.target_component = 1;
    c.source_system = 2; c.source_component = 1;
    c.from_external = true;
    cmd_pub_->publish(c);
  }

  void publish_task_origin(bool forced)
  {
    geometry_msgs::msg::PointStamped to;
    to.header.stamp = now();
    to.header.frame_id = "pursuer_local_ned";
    to.point.x = lp_->x; to.point.y = lp_->y; to.point.z = lp_->z;
    to_pub_->publish(to);
    to_pub_done_ = true;
    if (forced) {
      RCLCPP_WARN(
        get_logger(), "task origin FORCED after timeout [%.2f,%.2f,%.2f]",
        lp_->x, lp_->y, lp_->z);
    } else {
      RCLCPP_INFO(
        get_logger(), "task origin latched after convergence [%.2f,%.2f,%.2f]",
        lp_->x, lp_->y, lp_->z);
    }
  }

  // 航向设定点：首飞前初始化为当前航向，此后仅由伺服更新（锁定保持）
  double frozen_yaw()
  {
    if (!yaw_sp_init_ && lp_) {
      yaw_sp_ = lp_->heading;
      yaw_sp_init_ = true;
    }
    return yaw_sp_;
  }

  // 调试状态发布（2Hz）：phase/形态/授权/估计/目标速度/瞄准计时 +
  // 本机位姿/航向/站位目标/最近指令 + SPIN 进度。外场排查按字段对表
  // （见 docs/实物调试指南.md）。
  void publish_debug()
  {
    if (!lp_) {return;}
    const double t = now().seconds();
    const bool fcu_ok = fcu_inputs_fresh(t);
    bool est_ok = false;
    double tspd = 0.0;
    if (est_ && est_->valid) {
      const double age = t -
        est_->header.stamp.sec - est_->header.stamp.nanosec * 1e-9;
      est_ok = age >= -0.05 && age <= est_timeout_;
      if (est_ok) {
        tspd = std::hypot(est_->velocity.x, est_->velocity.y);
      }
    }
    std_msgs::msg::String msg;
    char buf[640];
    std::snprintf(buf, sizeof(buf),
      "phase=%s hold=%d bh=%d aim=%d auth=%d est=%d tspd=%.2f "
      "stab=%.1f aim_t=%.2f | pos=%.2f,%.2f,%.2f vel=%.2f,%.2f "
      "yaw=%.1f tgt=%.2f,%.2f,%.2f | spv=%.2f,%.2f spa=%.2f,%.2f "
      "spin=%.2f srch=%d to=%d fcu=%d lpms=%.0f stms=%.0f authms=%.0f",
      phase_name(phase_), static_cast<int>(hold_mode_),
      static_cast<int>(bearing_hold_ && !hold_mode_),
      static_cast<int>(phase_ == Phase::AIM),
      static_cast<int>(auth_effective(t)), static_cast<int>(est_ok), tspd,
      aim_stable_since_s_ > 0.0 ? t - aim_stable_since_s_ : -1.0,
      phase_ == Phase::AIM ? t - aim_since_s_ : -1.0,
      lp_->x, lp_->y, lp_->z, lp_->vx, lp_->vy,
      yaw_sp_ * 57.29577951308232,
      hold_target_[0], hold_target_[1], hold_target_[2],
      last_vx_, last_vy_, dbg_ax_, dbg_ay_,
      spin_accum_rad_,
      static_cast<int>(search_spin_enabled_ && !ever_had_est_ &&
        phase_ == Phase::TRACK),
      static_cast<int>(to_pub_done_),
      static_cast<int>(fcu_ok),
      lp_ ? (t - lp_rx_s_) * 1000.0 : -1.0,
      st_ ? (t - st_rx_s_) * 1000.0 : -1.0,
      (t - auth_rx_s_) * 1000.0);
    msg.data = buf;
    debug_pub_->publish(msg);
  }

  Phase phase_{Phase::WAITING};
  std::uint64_t ob_sent_{0};   // offboard 心跳计数（面板显示连续性）
  std::uint64_t sp_sent_{0};   // 轨迹设定点计数（待机时应停止增长）
  double last_alive_log_s_{-1e9};
  double start_s_{-1.0};
  double last_engage_s_{-100.0};
  int engage_attempts_{0};
  bool to_pub_done_{false};
  bool ready_published_{false};
  bool hold_mode_{true};
  bool shot_seen_{false};
  double speed_above_since_s_{-1.0};
  double track_entry_s_{-1.0};
  double standoff_target_m_{7.5};
  double accel_lead_s_{0.0};
  double integral_gain_{0.3};
  double integral_x_{0.0}, integral_y_{0.0};
  bool yaw_servo_enabled_{true};
  bool yaw_sp_init_{false};
  bool yaw_locked_{false};
  double yaw_sp_{0.0};
  double yaw_course_offset_rad_{0.0};
  double yaw_max_rate_{3.0};
  double yaw_above_since_s_{-1.0};
  bool bearing_hold_{false};
  bool bearing_hold_enabled_{true};
  double bearing_hold_yaw_{0.0};
  double bearing_hold_since_s_{0.0};
  double bearing_hold_sweep_rad_{0.44};
  double bearing_hold_sweep_period_s_{8.0};
  double bearing_hold_recover_confirm_s_{0.3};
  double bearing_hold_recover_speed_mps_{1.0};
  double bearing_hold_recover_timeout_s_{2.0};
  double bearing_hold_max_duration_s_{20.0};
  double est_lost_since_s_{-1.0};
  double est_recover_since_s_{-1.0};
  bool bh_closing_{false};
  double approach_speed_mps_{3.5};
  bool bearing_servo_enabled_{true};
  double bearing_servo_gain_{0.25};
  double bearing_servo_deadband_m_{0.15};
  double est_lp_tau_{0.0};
  double est_lp_[3]{0.0, 0.0, 0.0};
  bool est_lp_seeded_{false};  // R0-3：低通种子标志（第一条有效估计直赋）
  std::string engage_mode_{"auto"};
  bool allow_auto_engage_{false};
  int vehicle_command_target_system_{2};
  double fcu_timeout_s_{0.5};
  double weapon_auth_timeout_s_{0.5};
  double muzzle_speed_mps_{55.0};
  double deployment_time_s_{0.055};
  NetgunModel aim_model_;
  std::string target_velocity_frame_{"relative"};
  bool yaw_course_bidirectional_{false};
  double max_vertical_acc_{2.5};
  double max_lateral_acc_{0.0};
  double lp_rx_s_{-1e9};
  double st_rx_s_{-1e9};
  double auth_rx_s_{-1e9};
  bool lp_valid_{false};
  double last_reset_log_s_{-1e9};
  bool task_origin_wait_converged_{true};
  double task_origin_settle_s_{2.0};
  double task_origin_drift_max_m_{0.3};
  double task_origin_timeout_s_{20.0};
  double to_conv_since_s_{-1.0};
  double to_probe_[2]{0.0, 0.0};
  double intercept_speed_mps_{8.0};
  double bearing_hold_arrive_radius_m_{2.0};
  bool bearing_hold_intercept_nav_{false};
  double bh_abs_[2]{0.0, 0.0};
  double bh_vel_[2]{0.0, 0.0};
  double bh_time_{0.0};
  // AIM / SPIN（清单§八-0）
  bool aim_mode_enabled_{true};
  double aim_enter_delay_s_{2.0};
  double aim_max_tilt_rad_{0.35};
  double aim_timeout_s_{0.8};
  double aim_max_target_speed_mps_{100.0};
  double aim_tilt_compensation_{1.15};
  double aim_stable_since_s_{-1.0};
  double aim_since_s_{0.0};
  bool aim_seeded_{false};
  double aim_vx_{0.0}, aim_vy_{0.0};
  bool weapon_auth_{false};
  bool spin_flag_enabled_{true};
  double spin_rate_rad_s_{1.8};
  double spin_total_rad_{6.2832};
  double spin_accum_rad_{0.0};
  double spin_settle_speed_mps_{0.25};
  double spin_settle_confirm_s_{0.3};
  double spin_settle_timeout_s_{4.0};
  double shot_time_s_{0.0};
  double settle_ok_since_s_{-1.0};
  bool search_spin_enabled_{true};
  double search_spin_rate_rad_s_{0.42};
  bool ever_had_est_{false};
  bool search_spin_logged_{false};
  double yaw_engage_speed_mps_{0.5};
  double hold_target_[3]{0.0, 0.0, 0.0};  // [2] 构造后即设为 -takeoff_h_
  double last_vx_{0}, last_vy_{0}, last_vz_{0};
  double dbg_ax_{0.0}, dbg_ay_{0.0};
  double takeoff_h_, min_h_, gain_, max_spd_, max_acc_, ff_, v_gain_, max_vz_;
  double engage_delay_, est_timeout_, ctrl_hz_;
  double track_engage_speed_mps_{0.3};
  double prediction_horizon_s_{0.2};
  double max_pred_offset_m_{2.0};

  px4_offboard_tracking::msg::RelativeEstimate::ConstSharedPtr est_;
  px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr lp_;
  px4_msgs::msg::VehicleStatus::ConstSharedPtr st_;

  rclcpp::Subscription<px4_offboard_tracking::msg::RelativeEstimate>::SharedPtr est_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lp_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr st_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ob_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr to_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<virtual_interception_interfaces::msg::VirtualShotEvent>::SharedPtr shot_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace netgun_intercept

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<netgun_intercept::NetgunGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
