#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <virtual_interception_interfaces/msg/virtual_hit_result.hpp>
#include <virtual_interception_interfaces/msg/virtual_shot_event.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "virtual_interception_referee/referee_model.hpp"

namespace virtual_interception_referee
{

namespace
{
std::array<double, 3> interpolate_ned(
  const std::array<double, 3> & start, const std::array<double, 3> & end, double fraction)
{
  return {
    start[0] + (end[0] - start[0]) * fraction,
    start[1] + (end[1] - start[1]) * fraction,
    start[2] + (end[2] - start[2]) * fraction};
}
}  // namespace

class VirtualProjectileReferee : public rclcpp::Node
{
public:
  VirtualProjectileReferee()
  : Node("virtual_projectile_referee"),
    shot_topic_(declare_parameter<std::string>(
      "shot_topic", "/tracking/virtual_shot_event")),
    result_topic_(declare_parameter<std::string>(
      "result_topic", "/evaluation/virtual_hit_result")),
    solver_result_topic_(declare_parameter<std::string>(
      "solver_result_topic", "/evaluation/virtual_solver_hit_result")),
    truth_topic_(declare_parameter<std::string>(
      "truth_topic", "/evaluation/relative_truth")),
    task_origin_topic_(declare_parameter<std::string>(
      "task_origin_topic", "/tracking/task_origin_ned")),
    pursuer_local_topic_(declare_parameter<std::string>(
      "pursuer_local_topic", "/px4_1/fmu/out/vehicle_local_position")),
    pursuer_attitude_topic_(declare_parameter<std::string>(
      "pursuer_attitude_topic", "/px4_1/fmu/out/vehicle_attitude")),
    maximum_state_age_s_(declare_parameter<double>("maximum_state_age_s", 0.1)),
    maximum_truth_gap_s_(declare_parameter<double>("maximum_truth_gap_s", 0.1)),
    post_intercept_margin_s_(
      declare_parameter<double>("post_intercept_margin_s", 0.05)),
    effect_reference_range_m_(
      declare_parameter<double>("effect_reference_range_m", 35.0)),
    effect_reference_diameter_m_(
      declare_parameter<double>("effect_reference_diameter_m", 1.7)),
    model_(RefereeConfig{
      declare_parameter<double>("projectile_speed_mps", 400.0),
      declare_parameter<double>("gravity_mps2", 9.80665),
      declare_parameter<double>("hit_radius_m", 0.4),
      declare_parameter<double>("maximum_flight_time_s", 0.25),
      declare_parameter<double>("integration_step_s", 0.0001),
      std::atan(
        0.5 * effect_reference_diameter_m_ / effect_reference_range_m_),
      // 网枪 3×3 即时覆盖模式（方案 v4 §5）：默认关，网枪链显式开启。
      declare_parameter<bool>("instant_coverage_mode", false),
      declare_parameter<double>("coverage_half_extent_m", 1.5),
      declare_parameter<double>("effective_range_min_m", 6.0),
      declare_parameter<double>("effective_range_max_m", 10.0)}),
    mission_counter_(MissionHitCounterConfig{
      declare_parameter<double>("minimum_hit_interval_s", 3.0),
      static_cast<std::uint32_t>(declare_parameter<int>("required_hits", 2))})
  {
    if (maximum_state_age_s_ <= 0.0 || maximum_truth_gap_s_ <= 0.0 ||
      post_intercept_margin_s_ < 0.0 || effect_reference_range_m_ <= 0.0 ||
      effect_reference_diameter_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid referee timing configuration");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      pursuer_local_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
        if (!message->xy_valid || !message->z_valid) {
          return;
        }
        pursuer_local_position_ = {message->x, message->y, message->z};
        local_update_ = now();
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      pursuer_attitude_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {
        const std::array<double, 4> quaternion{
          message->q[0], message->q[1], message->q[2], message->q[3]};
        const auto axis = body_minus_z_in_ned(quaternion);
        if (!std::isfinite(axis[0]) || !std::isfinite(axis[1]) ||
          !std::isfinite(axis[2]))
        {
          return;
        }
        actual_attack_axis_ned_ = axis;
        attitude_update_ = now();
      });
    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    origin_qos.reliable().transient_local();
    task_origin_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      task_origin_topic_, origin_qos,
      [this](geometry_msgs::msg::PointStamped::UniquePtr message) {
        const std::array<double, 3> origin{
          message->point.x, message->point.y, message->point.z};
        if (!std::isfinite(origin[0]) || !std::isfinite(origin[1]) ||
          !std::isfinite(origin[2]))
        {
          return;
        }
        task_origin_ned_ = origin;
        task_origin_valid_ = true;
      });
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_topic_, 20,
      [this](nav_msgs::msg::Odometry::UniquePtr message) {
        on_truth(*message);
      });
    shot_sub_ =
      create_subscription<virtual_interception_interfaces::msg::VirtualShotEvent>(
      shot_topic_, 10,
      [this](
        virtual_interception_interfaces::msg::VirtualShotEvent::UniquePtr message)
      {
        on_shot(*message);
      });
    result_pub_ =
      create_publisher<virtual_interception_interfaces::msg::VirtualHitResult>(
      result_topic_, 10);
    solver_result_pub_ =
      create_publisher<virtual_interception_interfaces::msg::VirtualHitResult>(
      solver_result_topic_, 10);
    auto completion_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    completion_qos.reliable().transient_local();
    mission_success_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/evaluation/mission_success", completion_qos);
    truth_watchdog_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {check_truth_timeout();});
  }

private:
  struct ActiveShot
  {
    std::uint64_t id{};
    rclcpp::Time stamp;
    std::string frame_id;
    double predicted_time_of_flight_s{};
    RefereeShot physical_shot;
    RefereeShot solver_shot;
    double last_sample_time_s{-1.0};
    std::array<double, 3> last_target_position{};
    RefereeEvaluation physical_best;
    RefereeEvaluation solver_best;
  };

  bool state_fresh(const rclcpp::Time & update) const
  {
    if (update.nanoseconds() <= 0) {
      return false;
    }
    const double age = (now() - update).seconds();
    return age >= 0.0 && age <= maximum_state_age_s_;
  }

  bool truth_frame_ready() const
  {
    return task_origin_valid_ && state_fresh(local_update_);
  }

  bool shot_state_ready() const
  {
    return truth_frame_ready() && state_fresh(attitude_update_);
  }

  std::array<double, 3> current_target_position(
    const nav_msgs::msg::Odometry & truth) const
  {
    const auto pursuer_task_position = position_in_task_frame(
      pursuer_local_position_, task_origin_ned_);
    return {
      pursuer_task_position[0] + truth.pose.pose.position.x,
      pursuer_task_position[1] + truth.pose.pose.position.y,
      pursuer_task_position[2] + truth.pose.pose.position.z};
  }

  void on_shot(
    const virtual_interception_interfaces::msg::VirtualShotEvent & message)
  {
    RCLCPP_INFO(
      get_logger(),
      "referee on_shot: id=%lu speed=%.1f tof=%.3f solution=%d state_ready=%d",
      static_cast<unsigned long>(message.shot_id), message.projectile_speed_mps,
      message.predicted_time_of_flight_s,
      static_cast<int>(message.solution_valid),
      static_cast<int>(shot_state_ready()));
    if (message.interface_version !=
      virtual_interception_interfaces::msg::VirtualShotEvent::INTERFACE_VERSION_V1 ||
      !message.solution_valid || !shot_state_ready() ||
      message.projectile_speed_mps <= 0.0 ||
      message.shot_id == 0 ||
      message.shot_id <= last_accepted_shot_id_ ||
      rclcpp::Time(message.header.stamp).nanoseconds() <= 0 ||
      !std::isfinite(message.predicted_time_of_flight_s) ||
      message.predicted_time_of_flight_s <= 0.0 ||
      message.predicted_time_of_flight_s >
      model_.config().maximum_flight_time_s ||
      std::abs(message.projectile_speed_mps -
      model_.config().projectile_speed_mps) > 1e-6)
    {
      publish_invalid_pair(
        message.shot_id, "invalid shot event or unavailable task/attitude state",
        message.header.frame_id);
      return;
    }
    if (active_shot_.has_value()) {
      publish_invalid_pair(
        message.shot_id, "another virtual projectile is active",
        message.header.frame_id);
      return;
    }
    const std::array<double, 3> direction{
      message.shot_direction_ned.x,
      message.shot_direction_ned.y,
      message.shot_direction_ned.z};
    const double direction_norm = std::sqrt(
      direction[0] * direction[0] +
      direction[1] * direction[1] +
      direction[2] * direction[2]);
    if (!std::isfinite(direction_norm) || std::abs(direction_norm - 1.0) > 1e-3) {
      publish_invalid_pair(
        message.shot_id, "shot direction is not a unit vector",
        message.header.frame_id);
      return;
    }

    ActiveShot active;
    active.id = message.shot_id;
    active.stamp = rclcpp::Time(message.header.stamp);
    active.frame_id = message.header.frame_id;
    active.predicted_time_of_flight_s = message.predicted_time_of_flight_s;
    // 发射原点：用裁判自身订阅的追踪机本地位置换算 task 帧，与
    // current_target_position() 同帧。旧实现取消息字段 pursuer_position_ned，
    // 但 netgun strike 不填该字段（默认 0）→ 原点 (0,0,0)、目标 ~12m 外，
    // 幻影脱靶 1.3-2.1m 且距离门必判失败（2026-10-12 bag 比对定位）。
    const auto origin_task = position_in_task_frame(
      pursuer_local_position_, task_origin_ned_);
    active.physical_shot.origin_ned = {
      origin_task[0], origin_task[1], origin_task[2]};
    active.physical_shot.pursuer_velocity_ned = {
      message.pursuer_velocity_ned.x,
      message.pursuer_velocity_ned.y,
      message.pursuer_velocity_ned.z};
    active.physical_shot.direction_ned = actual_attack_axis_ned_;
    active.solver_shot = active.physical_shot;
    active.solver_shot.direction_ned = direction;
    active.physical_best.minimum_miss_distance_m =
      std::numeric_limits<double>::infinity();
    active.solver_best.minimum_miss_distance_m =
      std::numeric_limits<double>::infinity();
    active_shot_ = active;
    last_accepted_shot_id_ = message.shot_id;
  }

  void on_truth(const nav_msgs::msg::Odometry & message)
  {
    if (!truth_frame_ready()) {
      return;
    }
    const rclcpp::Time sample_stamp(message.header.stamp);
    const auto sample_target_ned = current_target_position(message);
    if (!active_shot_.has_value()) {
      // Keep the latest target position as the pre-shot anchor.
      pre_shot_valid_ = true;
      pre_shot_stamp_ = sample_stamp;
      pre_shot_target_ned_ = sample_target_ned;
      return;
    }
    auto & active = *active_shot_;
    const double elapsed = (sample_stamp - active.stamp).seconds();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "referee trace: active id=%lu elapsed=%.3f last_sample=%.3f",
      static_cast<unsigned long>(active.id), elapsed, active.last_sample_time_s);
    if (!std::isfinite(elapsed)) {
      return;
    }
    const auto target = current_target_position(message);
    if (active.last_sample_time_s < 0.0) {
      if (!truth_interval_usable(0.0, elapsed, maximum_truth_gap_s_)) {
        publish_invalid_pair(
          active.id, "truth unavailable across shot start", active.frame_id);
        active_shot_.reset();
        return;
      }
      // Score the initial [0, e1] segment: interpolate the target position
      // at the shot instant between the retained pre-shot sample and this
      // first post-shot sample. Without it a 400 m/s projectile covers its
      // whole close-range flight before the second sample arrives, and
      // hits can never be scored.
      if (pre_shot_valid_) {
        const double e_pre = (pre_shot_stamp_ - active.stamp).seconds();
        if (std::isfinite(e_pre) && elapsed > e_pre) {
          const double fraction0 = (0.0 - e_pre) / (elapsed - e_pre);
          const auto target_at_shot = interpolate_ned(
            pre_shot_target_ned_, target, fraction0);
          const RefereeSegment initial_segment{
            0.0, elapsed, target_at_shot, target};
          const auto initial_physical = model_.evaluate_segment(
            active.physical_shot, initial_segment);
          if (initial_physical.valid &&
            initial_physical.minimum_miss_distance_m <
            active.physical_best.minimum_miss_distance_m)
          {
            active.physical_best = initial_physical;
          }
          const auto initial_solver = model_.evaluate_segment(
            active.solver_shot, initial_segment);
          if (initial_solver.valid &&
            initial_solver.minimum_miss_distance_m <
            active.solver_best.minimum_miss_distance_m)
          {
            active.solver_best = initial_solver;
          }
        }
      }
      active.last_sample_time_s = elapsed;
      active.last_target_position = target;
      return;
    }

    if (!truth_interval_usable(
        active.last_sample_time_s, elapsed, maximum_truth_gap_s_))
    {
      publish_invalid_pair(
        active.id, "truth gap exceeded during projectile flight", active.frame_id);
      active_shot_.reset();
      return;
    }

    const double bounded_elapsed = std::min(
      elapsed, model_.config().maximum_flight_time_s);
    const RefereeSegment segment{
      active.last_sample_time_s, bounded_elapsed,
      active.last_target_position, target};
    const auto physical_evaluation = model_.evaluate_segment(
      active.physical_shot, segment);
    if (physical_evaluation.valid &&
      physical_evaluation.minimum_miss_distance_m <
      active.physical_best.minimum_miss_distance_m)
    {
      active.physical_best = physical_evaluation;
    }
    const auto solver_evaluation = model_.evaluate_segment(
      active.solver_shot, segment);
    if (solver_evaluation.valid &&
      solver_evaluation.minimum_miss_distance_m <
      active.solver_best.minimum_miss_distance_m)
    {
      active.solver_best = solver_evaluation;
    }
    active.last_sample_time_s = bounded_elapsed;
    active.last_target_position = target;

    const double decision_time = std::min(
      model_.config().maximum_flight_time_s,
      active.predicted_time_of_flight_s + post_intercept_margin_s_);
    if (elapsed >= decision_time) {
      RCLCPP_INFO(
        get_logger(), "referee trace: decision reached elapsed=%.3f dt=%.3f",
        elapsed, decision_time);
      publish_result(active);
      active_shot_.reset();
    }
  }

  void check_truth_timeout()
  {
    if (!active_shot_.has_value()) {
      return;
    }
    const auto & active = *active_shot_;
    const double elapsed = (now() - active.stamp).seconds();
    const double last_sample = std::max(0.0, active.last_sample_time_s);
    if (!std::isfinite(elapsed) || elapsed < last_sample ||
      elapsed - last_sample <= maximum_truth_gap_s_)
    {
      return;
    }
    publish_invalid_pair(
      active.id, "truth watchdog expired during projectile flight",
      active.frame_id);
    active_shot_.reset();
  }

  void publish_invalid(
    const rclcpp::Publisher<
      virtual_interception_interfaces::msg::VirtualHitResult>::SharedPtr & publisher,
    std::uint64_t shot_id, const std::string & reason,
    const std::string & frame_id)
  {
    virtual_interception_interfaces::msg::VirtualHitResult output;
    output.interface_version =
      virtual_interception_interfaces::msg::VirtualHitResult::INTERFACE_VERSION_V1;
    output.header.stamp = now();
    output.header.frame_id = frame_id.empty() ? "pursuer_local_ned" : frame_id;
    output.shot_id = shot_id;
    output.result =
      virtual_interception_interfaces::msg::VirtualHitResult::RESULT_INVALID;
    output.hit = false;
    output.hit_radius_m = std::numeric_limits<double>::quiet_NaN();
    output.minimum_miss_distance_m = std::numeric_limits<double>::quiet_NaN();
    output.time_to_closest_approach_s = std::numeric_limits<double>::quiet_NaN();
    output.reason = reason;
    publisher->publish(output);
  }

  void publish_invalid_pair(
    std::uint64_t shot_id, const std::string & reason,
    const std::string & frame_id)
  {
    publish_invalid(result_pub_, shot_id, "physical-axis: " + reason, frame_id);
    publish_invalid(
      solver_result_pub_, shot_id, "solver-direction: " + reason, frame_id);
  }

  bool publish_evaluation(
    const rclcpp::Publisher<
      virtual_interception_interfaces::msg::VirtualHitResult>::SharedPtr & publisher,
    const ActiveShot & active, const RefereeEvaluation & evaluation,
    const std::string & label)
  {
    virtual_interception_interfaces::msg::VirtualHitResult output;
    output.interface_version =
      virtual_interception_interfaces::msg::VirtualHitResult::INTERFACE_VERSION_V1;
    output.header.stamp = now();
    output.header.frame_id = active.frame_id.empty() ?
      "pursuer_local_ned" : active.frame_id;
    output.shot_id = active.id;
    output.hit = evaluation.valid && evaluation.hit;
    output.result = !evaluation.valid ?
      virtual_interception_interfaces::msg::VirtualHitResult::RESULT_INVALID :
      (output.hit ?
      virtual_interception_interfaces::msg::VirtualHitResult::RESULT_HIT :
      virtual_interception_interfaces::msg::VirtualHitResult::RESULT_MISS);
    output.hit_radius_m = evaluation.valid ?
      evaluation.effective_hit_radius_m :
      std::numeric_limits<double>::quiet_NaN();
    output.minimum_miss_distance_m = evaluation.valid ?
      evaluation.minimum_miss_distance_m :
      std::numeric_limits<double>::quiet_NaN();
    output.time_to_closest_approach_s = evaluation.valid ?
      evaluation.closest_time_s : std::numeric_limits<double>::quiet_NaN();
    output.projectile_closest_position_ned.x =
      evaluation.projectile_closest_ned[0];
    output.projectile_closest_position_ned.y =
      evaluation.projectile_closest_ned[1];
    output.projectile_closest_position_ned.z =
      evaluation.projectile_closest_ned[2];
    output.target_closest_position_ned.x =
      evaluation.target_closest_ned[0];
    output.target_closest_position_ned.y =
      evaluation.target_closest_ned[1];
    output.target_closest_position_ned.z =
      evaluation.target_closest_ned[2];
    output.reason = !evaluation.valid ? label + ": no valid truth segment" :
      (output.hit ? label + ": truth miss distance inside hit radius" :
      label + ": truth miss distance outside hit radius");
    publisher->publish(output);
    return output.hit;
  }

  void publish_result(const ActiveShot & active)
  {
    publish_evaluation(
      solver_result_pub_, active, active.solver_best, "solver-direction");
    const bool physical_hit = publish_evaluation(
      result_pub_, active, active.physical_best, "physical-axis");
    RCLCPP_INFO(
      get_logger(), "REFEREE RESULT shot=%lu hit=%d miss=%.2fm valid=%d",
      static_cast<unsigned long>(active.id), static_cast<int>(physical_hit),
      active.physical_best.minimum_miss_distance_m,
      static_cast<int>(active.physical_best.valid));
    if (physical_hit) {
      if (!mission_counter_.accept(true, active.stamp.seconds())) {
        RCLCPP_WARN(
          get_logger(),
          "physical hit %lu not counted by mission interval rule",
          static_cast<unsigned long>(active.id));
      }
    }
    std_msgs::msg::Bool mission_success;
    mission_success.data = mission_counter_.mission_success();
    mission_success_pub_->publish(mission_success);
  }

  std::string shot_topic_;
  std::string result_topic_;
  std::string solver_result_topic_;
  std::string truth_topic_;
  std::string task_origin_topic_;
  std::string pursuer_local_topic_;
  std::string pursuer_attitude_topic_;
  double maximum_state_age_s_;
  double maximum_truth_gap_s_;
  double post_intercept_margin_s_;
  double effect_reference_range_m_;
  double effect_reference_diameter_m_;
  RefereeModel model_;
  MissionHitCounter mission_counter_;
  std::array<double, 3> pursuer_local_position_{};
  std::array<double, 3> task_origin_ned_{};
  std::array<double, 3> actual_attack_axis_ned_{};
  bool task_origin_valid_{false};
  // Latest pre-shot truth anchor: lets the referee score the initial
  // [0, e1] segment of a shot (a 400 m/s projectile finishes a 5-10 m
  // flight before the first post-shot truth sample arrives).
  bool pre_shot_valid_{false};
  rclcpp::Time pre_shot_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 3> pre_shot_target_ned_{};
  std::optional<ActiveShot> active_shot_;
  std::uint64_t last_accepted_shot_id_{0};
  rclcpp::Time local_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time attitude_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<
    virtual_interception_interfaces::msg::VirtualShotEvent>::SharedPtr shot_sub_;
  rclcpp::Publisher<
    virtual_interception_interfaces::msg::VirtualHitResult>::SharedPtr result_pub_;
  rclcpp::Publisher<
    virtual_interception_interfaces::msg::VirtualHitResult>::SharedPtr solver_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_success_pub_;
  rclcpp::TimerBase::SharedPtr truth_watchdog_;
};

}  // namespace virtual_interception_referee

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      virtual_interception_referee::VirtualProjectileReferee>());
  rclcpp::shutdown();
  return 0;
}
