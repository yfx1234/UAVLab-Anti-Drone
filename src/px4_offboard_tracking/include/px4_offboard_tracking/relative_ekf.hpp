#pragma once

#include <array>

#include <Eigen/Core>

namespace px4_offboard_tracking
{

struct RelativeEkfConfig
{
  // Retained parameter name for launch-file compatibility. In the constant-
  // acceleration model this drives the white-jerk process noise.
  double acceleration_noise_stddev_mps2{2.0};
  double initial_position_stddev_m{1.0};
  double initial_velocity_stddev_mps{3.0};
  double innovation_gate_mahalanobis{11.345};  // 99% chi-square threshold, 3 DoF.
  double initial_acceleration_stddev_mps2{3.0};
  // CT（协同转弯）模型过程噪声与初值（方案 v4 WP-D）。
  double turn_rate_process_noise_stddev_rads{0.5};
  double initial_turn_rate_stddev_rads{0.3};
};

struct RelativeEkfState
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  std::array<double, 36> covariance{};
  std::array<double, 81> full_covariance{};
  std::array<double, 3> innovation{};
  // [cv, ca, ct]（三模 IMM，方案 v4 WP-D；旧 [cv, ca] 消费方按索引取
  // [1]=CA 概率的语义不变）。
  std::array<double, 3> model_probabilities{{1.0, 0.0, 0.0}};
  double innovation_mahalanobis{};
  double timestamp_seconds{};
  double last_accepted_measurement_seconds{};
  bool measurement_accepted{false};
  bool initialized{false};
};

class RelativeEkf
{
public:
  explicit RelativeEkf(const RelativeEkfConfig & config);

  bool update(
    const std::array<double, 3> & position_ned,
    const std::array<double, 9> & measurement_covariance,
    double timestamp_seconds);

  RelativeEkfState state() const;

  bool valid(double current_time_seconds, double max_data_age_s, double max_position_variance) const;

private:
  void predict(double delta_time_s);

  RelativeEkfConfig config_;
  Eigen::Matrix<double, 9, 1> state_vector_{Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Matrix<double, 9, 9> covariance_{Eigen::Matrix<double, 9, 9>::Identity()};
  Eigen::Matrix<double, 3, 1> innovation_{Eigen::Matrix<double, 3, 1>::Zero()};
  double innovation_mahalanobis_{};
  double timestamp_seconds_{};
  double last_accepted_measurement_seconds_{};
  bool last_measurement_accepted_{false};
  bool initialized_{false};
};

struct RelativeImmEkfConfig
{
  RelativeEkfConfig filter{};
  double probability_cv_to_cv{0.97};
  double probability_ca_to_ca{0.95};
  double probability_ct_to_ct{0.95};
  double initial_cv_probability{0.6};
  double initial_ct_probability{0.15};
  // 初始 CA 概率 = 1 - initial_cv_probability - initial_ct_probability。
};

/// 运动模型种类：匀速 / 匀加速 / 协同转弯（水平面，含垂直 CA 通道）。
enum class RelativeMotionModel
{
  CONSTANT_VELOCITY = 0,
  CONSTANT_ACCELERATION = 1,
  COORDINATED_TURN = 2,
};

class RelativeImmEkf
{
public:
  explicit RelativeImmEkf(const RelativeImmEkfConfig & config);

  bool update(
    const std::array<double, 3> & position_ned,
    const std::array<double, 9> & measurement_covariance,
    double timestamp_seconds);

  RelativeEkfState state() const;
  RelativeEkfState predicted_state(double timestamp_seconds) const;
  bool valid(double current_time_seconds, double max_data_age_s, double max_position_variance) const;
  std::array<double, 3> model_probabilities() const;

  // 发散恢复：清空初始化标志，下一条通过校验的观测将重新初始化滤波器
  //（快速交叉目标下 IMM 速度估计可能爆掉，随后革新门把好观测全部拒收
  // 形成死锁——连续拒收超过阈值后由节点层调用本方法强制重置）。
  void reset() {initialized_ = false;}

private:
  // 内部状态 10 维：[p(3) v(3) a(3) ω(1)]；ω 仅 CT 模型驱动。
  struct ModelState
  {
    Eigen::Matrix<double, 10, 1> x{Eigen::Matrix<double, 10, 1>::Zero()};
    Eigen::Matrix<double, 10, 10> p{Eigen::Matrix<double, 10, 10>::Identity()};
    Eigen::Vector3d innovation{Eigen::Vector3d::Zero()};
    double mahalanobis{};
  };

  void mix_models();
  void predict(ModelState & model, double dt, RelativeMotionModel kind) const;
  bool correct(
    ModelState & model, const Eigen::Vector3d & measurement,
    const Eigen::Matrix3d & measurement_noise, double & likelihood) const;
  void combine();

  RelativeImmEkfConfig config_;
  std::array<ModelState, 3> models_{};
  std::array<double, 3> probabilities_{{0.6, 0.25, 0.15}};
  RelativeEkfState combined_{};
  double timestamp_seconds_{};
  double last_accepted_measurement_seconds_{};
  bool last_measurement_accepted_{false};
  bool initialized_{false};
};

}  // namespace px4_offboard_tracking
