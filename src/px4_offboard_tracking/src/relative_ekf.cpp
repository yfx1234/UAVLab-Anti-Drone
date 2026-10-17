#include "px4_offboard_tracking/relative_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/LU>


namespace px4_offboard_tracking
{

RelativeEkf::RelativeEkf(const RelativeEkfConfig & config)
: config_(config)
{
  if (config_.acceleration_noise_stddev_mps2 < 0.0 ||
    config_.initial_position_stddev_m <= 0.0 ||
    config_.initial_velocity_stddev_mps <= 0.0 ||
    config_.initial_acceleration_stddev_mps2 <= 0.0 ||
    config_.innovation_gate_mahalanobis <= 0.0)
  {
    throw std::invalid_argument("invalid relative EKF configuration");
  }
}

void RelativeEkf::predict(double delta_time_s)
{
  Eigen::Matrix<double, 9, 9> transition = Eigen::Matrix<double, 9, 9>::Identity();
  transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_time_s;
  transition.block<3, 3>(0, 6) =
    Eigen::Matrix3d::Identity() * (0.5 * delta_time_s * delta_time_s);
  transition.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * delta_time_s;

  const double dt2 = delta_time_s * delta_time_s;
  const double dt3 = dt2 * delta_time_s;
  const double dt4 = dt2 * dt2;
  const double dt5 = dt4 * delta_time_s;
  const double jerk_variance =
    config_.acceleration_noise_stddev_mps2 * config_.acceleration_noise_stddev_mps2;
  Eigen::Matrix<double, 9, 9> process_noise = Eigen::Matrix<double, 9, 9>::Zero();
  process_noise.block<3, 3>(0, 0) =
    Eigen::Matrix3d::Identity() * (0.05 * dt5 * jerk_variance);
  process_noise.block<3, 3>(0, 3) =
    Eigen::Matrix3d::Identity() * (0.125 * dt4 * jerk_variance);
  process_noise.block<3, 3>(3, 0) = process_noise.block<3, 3>(0, 3);
  process_noise.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity() * (dt3 * jerk_variance / 3.0);
  process_noise.block<3, 3>(0, 6) =
    Eigen::Matrix3d::Identity() * (dt3 * jerk_variance / 6.0);
  process_noise.block<3, 3>(6, 0) = process_noise.block<3, 3>(0, 6);
  process_noise.block<3, 3>(3, 6) =
    Eigen::Matrix3d::Identity() * (0.5 * dt2 * jerk_variance);
  process_noise.block<3, 3>(6, 3) = process_noise.block<3, 3>(3, 6);
  process_noise.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() * (delta_time_s * jerk_variance);

  state_vector_ = transition * state_vector_;
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
}

bool RelativeEkf::update(
  const std::array<double, 3> & position_ned,
  const std::array<double, 9> & measurement_covariance,
  double timestamp_seconds)
{
  for (const double value : position_ned) {
    if (!std::isfinite(value)) {return false;}
  }
  Eigen::Matrix3d measurement_noise;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double value = measurement_covariance[3 * row + column];
      if (!std::isfinite(value)) {return false;}
      measurement_noise(row, column) = value;
    }
  }
  if (!std::isfinite(timestamp_seconds) || measurement_noise.diagonal().minCoeff() <= 0.0) {
    return false;
  }

  const Eigen::Vector3d measurement(position_ned[0], position_ned[1], position_ned[2]);
  if (!initialized_) {
    state_vector_.setZero();
    state_vector_.head<3>() = measurement;
    covariance_.setZero();
    covariance_.diagonal().head<3>().setConstant(
      config_.initial_position_stddev_m * config_.initial_position_stddev_m);
    covariance_.diagonal().tail<3>().setConstant(
      config_.initial_acceleration_stddev_mps2 * config_.initial_acceleration_stddev_mps2);
    covariance_.diagonal().segment<3>(3).setConstant(
      config_.initial_velocity_stddev_mps * config_.initial_velocity_stddev_mps);
    timestamp_seconds_ = timestamp_seconds;
    last_accepted_measurement_seconds_ = timestamp_seconds;
    innovation_.setZero();
    innovation_mahalanobis_ = 0.0;
    last_measurement_accepted_ = true;
    initialized_ = true;
    return true;
  }

  const double delta_time = timestamp_seconds - timestamp_seconds_;
  if (delta_time <= 0.0) {
    last_measurement_accepted_ = false;
    return false;
  }
  predict(delta_time);
  timestamp_seconds_ = timestamp_seconds;

  Eigen::Matrix<double, 3, 9> observation = Eigen::Matrix<double, 3, 9>::Zero();
  observation.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  innovation_ = measurement - observation * state_vector_;
  const Eigen::Matrix3d innovation_covariance =
    observation * covariance_ * observation.transpose() + measurement_noise;
  const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) {return false;}
  innovation_mahalanobis_ = innovation_.dot(decomposition.solve(innovation_));
  if (!std::isfinite(innovation_mahalanobis_) ||
    innovation_mahalanobis_ > config_.innovation_gate_mahalanobis)
  {
    last_measurement_accepted_ = false;
    return false;
  }

  const Eigen::Matrix<double, 9, 3> gain =
    covariance_ * observation.transpose() * decomposition.solve(Eigen::Matrix3d::Identity());
  state_vector_ += gain * innovation_;
  const Eigen::Matrix<double, 9, 9> identity = Eigen::Matrix<double, 9, 9>::Identity();
  const auto correction = identity - gain * observation;
  covariance_ = correction * covariance_ * correction.transpose() +
    gain * measurement_noise * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  last_accepted_measurement_seconds_ = timestamp_seconds;
  last_measurement_accepted_ = true;
  return true;
}

bool RelativeEkf::valid(
  double current_time_seconds, double max_data_age_s, double max_position_variance) const
{
  if (!initialized_ || current_time_seconds < last_accepted_measurement_seconds_ ||
    current_time_seconds - last_accepted_measurement_seconds_ > max_data_age_s)
  {
    return false;
  }
  return covariance_.diagonal().head<3>().maxCoeff() <= max_position_variance &&
         state_vector_.allFinite() && covariance_.allFinite();
}

RelativeEkfState RelativeEkf::state() const
{
  RelativeEkfState output;
  output.initialized = initialized_;
  output.timestamp_seconds = timestamp_seconds_;
  output.last_accepted_measurement_seconds = last_accepted_measurement_seconds_;
  output.innovation_mahalanobis = innovation_mahalanobis_;
  output.measurement_accepted = last_measurement_accepted_;
  for (int axis = 0; axis < 3; ++axis) {
    output.position[axis] = state_vector_(axis);
    output.velocity[axis] = state_vector_(axis + 3);
    output.acceleration[axis] = state_vector_(axis + 6);
    output.innovation[axis] = innovation_(axis);
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      output.covariance[6 * row + column] = covariance_(row, column);
    }
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      output.full_covariance[9 * row + column] = covariance_(row, column);
    }
  }
  return output;
}

RelativeImmEkf::RelativeImmEkf(const RelativeImmEkfConfig & config)
: config_(config)
{
  const auto & f = config_.filter;
  const double initial_ca =
    1.0 - config_.initial_cv_probability - config_.initial_ct_probability;
  if (f.acceleration_noise_stddev_mps2 < 0.0 || f.initial_position_stddev_m <= 0.0 ||
    f.initial_velocity_stddev_mps <= 0.0 || f.initial_acceleration_stddev_mps2 <= 0.0 ||
    f.innovation_gate_mahalanobis <= 0.0 || f.turn_rate_process_noise_stddev_rads < 0.0 ||
    f.initial_turn_rate_stddev_rads <= 0.0 || config_.probability_cv_to_cv <= 0.0 ||
    config_.probability_cv_to_cv >= 1.0 || config_.probability_ca_to_ca <= 0.0 ||
    config_.probability_ca_to_ca >= 1.0 || config_.probability_ct_to_ct <= 0.0 ||
    config_.probability_ct_to_ct >= 1.0 || config_.initial_cv_probability <= 0.0 ||
    config_.initial_ct_probability < 0.0 || initial_ca <= 0.0)
  {
    throw std::invalid_argument("invalid relative IMM-EKF configuration");
  }
  probabilities_ = {
    config_.initial_cv_probability, initial_ca, config_.initial_ct_probability};
}

void RelativeImmEkf::mix_models()
{
  // 3×3 转移概率矩阵（行=源，列=目的）；对角参数化，非对角余量均分。
  const double transition[3][3] = {
    {config_.probability_cv_to_cv, (1.0 - config_.probability_cv_to_cv) * 0.5,
      (1.0 - config_.probability_cv_to_cv) * 0.5},
    {(1.0 - config_.probability_ca_to_ca) * 0.5, config_.probability_ca_to_ca,
      (1.0 - config_.probability_ca_to_ca) * 0.5},
    {(1.0 - config_.probability_ct_to_ct) * 0.5, (1.0 - config_.probability_ct_to_ct) * 0.5,
      config_.probability_ct_to_ct}};
  std::array<ModelState, 3> mixed;
  std::array<double, 3> normalizers{};
  for (int destination = 0; destination < 3; ++destination) {
    for (int source = 0; source < 3; ++source) {
      normalizers[destination] += probabilities_[source] * transition[source][destination];
    }
    for (int source = 0; source < 3; ++source) {
      const double weight = probabilities_[source] * transition[source][destination] /
        normalizers[destination];
      mixed[destination].x += weight * models_[source].x;
    }
    mixed[destination].p.setZero();
    for (int source = 0; source < 3; ++source) {
      const double weight = probabilities_[source] * transition[source][destination] /
        normalizers[destination];
      const auto delta = models_[source].x - mixed[destination].x;
      mixed[destination].p += weight * (models_[source].p + delta * delta.transpose());
    }
  }
  models_ = mixed;
  probabilities_ = normalizers;
}

void RelativeImmEkf::predict(ModelState & model, double dt, RelativeMotionModel kind) const
{
  Eigen::Matrix<double, 10, 10> transition = Eigen::Matrix<double, 10, 10>::Identity();
  transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  if (kind == RelativeMotionModel::CONSTANT_VELOCITY) {
    transition.block<3, 3>(6, 6).setZero();
  } else {
    transition.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
    transition.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
  }
  const double omega = model.x(9);
  if (kind == RelativeMotionModel::COORDINATED_TURN) {
    // 水平协同转弯一阶展开（NED x-y 平面）：θ=ω·dt 在 50Hz 更新下
    // (|ω|≤2 rad/s → θ≤0.04 rad)，一阶截断误差 <1e-3；精确圆弧积分留待
    // 数值仿真验证后按需替换（接口不变）。
    transition(3, 4) = -omega * dt;
    transition(4, 3) = omega * dt;
    transition(3, 9) = -dt * model.x(4);
    transition(4, 9) = dt * model.x(3);
  } else {
    // CV/CA 模型不驱动 ω；将 ω 状态与方差收敛回先验，避免无意义漂移。
    transition(9, 9) = 0.0;
    model.x(9) = 0.0;
    model.p(9, 9) = config_.filter.initial_turn_rate_stddev_rads *
      config_.filter.initial_turn_rate_stddev_rads;
  }
  const double variance = config_.filter.acceleration_noise_stddev_mps2 *
    config_.filter.acceleration_noise_stddev_mps2;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double dt5 = dt4 * dt;
  Eigen::Matrix<double, 10, 10> noise = Eigen::Matrix<double, 10, 10>::Zero();
  if (kind != RelativeMotionModel::CONSTANT_VELOCITY) {
    noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (dt5 * variance / 20.0);
    noise.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (dt4 * variance / 8.0);
    noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3);
    noise.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * (dt3 * variance / 6.0);
    noise.block<3, 3>(6, 0) = noise.block<3, 3>(0, 6);
    noise.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (dt3 * variance / 3.0);
    noise.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * (dt2 * variance / 2.0);
    noise.block<3, 3>(6, 3) = noise.block<3, 3>(3, 6);
    noise.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (dt * variance);
  } else {
    noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.25 * dt4 * variance);
    noise.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (0.5 * dt3 * variance);
    noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3);
    noise.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (dt2 * variance);
    noise.block<3, 3>(6, 6).setZero();
  }
  if (kind == RelativeMotionModel::COORDINATED_TURN) {
    noise(9, 9) = config_.filter.turn_rate_process_noise_stddev_rads *
      config_.filter.turn_rate_process_noise_stddev_rads * dt;
  }
  model.x = transition * model.x;
  model.p = transition * model.p * transition.transpose() + noise;
}

bool RelativeImmEkf::correct(
  ModelState & model, const Eigen::Vector3d & measurement,
  const Eigen::Matrix3d & measurement_noise, double & likelihood) const
{
  Eigen::Matrix<double, 3, 10> observation = Eigen::Matrix<double, 3, 10>::Zero();
  observation.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  model.innovation = measurement - observation * model.x;
  const Eigen::Matrix3d innovation_covariance =
    observation * model.p * observation.transpose() + measurement_noise;
  const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) {likelihood = 0.0; return false;}
  model.mahalanobis = model.innovation.dot(decomposition.solve(model.innovation));
  const double determinant = std::max(innovation_covariance.determinant(), 1e-18);
  likelihood = std::exp(-0.5 * std::min(model.mahalanobis, 100.0)) /
    std::sqrt(std::pow(2.0 * 3.14159265358979323846, 3) * determinant);
  if (!std::isfinite(model.mahalanobis) ||
    model.mahalanobis > config_.filter.innovation_gate_mahalanobis)
  {
    likelihood = 0.0;
    return false;
  }
  const Eigen::Matrix<double, 10, 3> gain = model.p * observation.transpose() *
    decomposition.solve(Eigen::Matrix3d::Identity());
  model.x += gain * model.innovation;
  const auto identity = Eigen::Matrix<double, 10, 10>::Identity();
  const auto correction = identity - gain * observation;
  model.p = correction * model.p * correction.transpose() +
    gain * measurement_noise * gain.transpose();
  model.p = 0.5 * (model.p + model.p.transpose());
  return true;
}

void RelativeImmEkf::combine()
{
  Eigen::Matrix<double, 10, 1> mean = Eigen::Matrix<double, 10, 1>::Zero();
  for (int model = 0; model < 3; ++model) {mean += probabilities_[model] * models_[model].x;}
  Eigen::Matrix<double, 10, 10> covariance = Eigen::Matrix<double, 10, 10>::Zero();
  for (int model = 0; model < 3; ++model) {
    const auto delta = models_[model].x - mean;
    covariance += probabilities_[model] * (models_[model].p + delta * delta.transpose());
  }
  combined_.initialized = initialized_;
  combined_.timestamp_seconds = timestamp_seconds_;
  combined_.last_accepted_measurement_seconds = last_accepted_measurement_seconds_;
  combined_.innovation_mahalanobis = 0.0;
  for (int model = 0; model < 3; ++model) {
    combined_.innovation_mahalanobis += probabilities_[model] * models_[model].mahalanobis;
  }
  combined_.model_probabilities = probabilities_;
  combined_.measurement_accepted = last_measurement_accepted_;
  for (int axis = 0; axis < 3; ++axis) {
    combined_.position[axis] = mean(axis);
    combined_.velocity[axis] = mean(axis + 3);
    combined_.acceleration[axis] = mean(axis + 6);
    combined_.innovation[axis] = 0.0;
    for (int model = 0; model < 3; ++model) {
      combined_.innovation[axis] +=
        probabilities_[model] * models_[model].innovation(axis);
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      combined_.covariance[6 * row + column] = covariance(row, column);
    }
  }
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      combined_.full_covariance[9 * row + column] = covariance(row, column);
    }
  }
}

bool RelativeImmEkf::update(
  const std::array<double, 3> & position_ned,
  const std::array<double, 9> & measurement_covariance, double timestamp_seconds)
{
  Eigen::Vector3d measurement;
  Eigen::Matrix3d measurement_noise;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(position_ned[axis])) {return false;}
    measurement(axis) = position_ned[axis];
    for (int column = 0; column < 3; ++column) {
      const double value = measurement_covariance[3 * axis + column];
      if (!std::isfinite(value)) {return false;}
      measurement_noise(axis, column) = value;
    }
  }
  if (!std::isfinite(timestamp_seconds) || measurement_noise.diagonal().minCoeff() <= 0.0) {
    return false;
  }
  if (!initialized_) {
    for (auto & model : models_) {
      model.x.setZero();
      model.x.head<3>() = measurement;
      model.p.setZero();
      model.p.diagonal().head<3>().setConstant(
        config_.filter.initial_position_stddev_m * config_.filter.initial_position_stddev_m);
      model.p.diagonal().segment<3>(3).setConstant(
        config_.filter.initial_velocity_stddev_mps * config_.filter.initial_velocity_stddev_mps);
      model.p.diagonal().segment<3>(6).setConstant(
        config_.filter.initial_acceleration_stddev_mps2 *
        config_.filter.initial_acceleration_stddev_mps2);
      model.p(9, 9) = config_.filter.initial_turn_rate_stddev_rads *
        config_.filter.initial_turn_rate_stddev_rads;
    }
    timestamp_seconds_ = timestamp_seconds;
    last_accepted_measurement_seconds_ = timestamp_seconds;
    last_measurement_accepted_ = true;
    initialized_ = true;
    combine();
    return true;
  }
  const double dt = timestamp_seconds - timestamp_seconds_;
  if (dt <= 0.0) {
    last_measurement_accepted_ = false;
    combine();
    return false;
  }
  mix_models();
  predict(models_[0], dt, RelativeMotionModel::CONSTANT_VELOCITY);
  predict(models_[1], dt, RelativeMotionModel::CONSTANT_ACCELERATION);
  predict(models_[2], dt, RelativeMotionModel::COORDINATED_TURN);
  std::array<double, 3> likelihood{};
  std::array<bool, 3> accepted{};
  for (int model = 0; model < 3; ++model) {
    accepted[model] = correct(models_[model], measurement, measurement_noise, likelihood[model]);
  }
  double normalizer = 0.0;
  for (int model = 0; model < 3; ++model) {
    normalizer += probabilities_[model] * likelihood[model];
  }
  if (normalizer > 1e-300 && std::isfinite(normalizer)) {
    for (int model = 0; model < 3; ++model) {
      probabilities_[model] = probabilities_[model] * likelihood[model] / normalizer;
    }
  }
  timestamp_seconds_ = timestamp_seconds;
  last_measurement_accepted_ = accepted[0] || accepted[1] || accepted[2];
  if (last_measurement_accepted_) {last_accepted_measurement_seconds_ = timestamp_seconds;}
  combine();
  return last_measurement_accepted_;
}

RelativeEkfState RelativeImmEkf::state() const {return combined_;}

RelativeEkfState RelativeImmEkf::predicted_state(double timestamp_seconds) const
{
  if (!initialized_ || !std::isfinite(timestamp_seconds) ||
    timestamp_seconds <= timestamp_seconds_)
  {
    return combined_;
  }
  RelativeImmEkf predicted = *this;
  const double dt = timestamp_seconds - predicted.timestamp_seconds_;
  predicted.mix_models();
  predicted.predict(predicted.models_[0], dt, RelativeMotionModel::CONSTANT_VELOCITY);
  predicted.predict(predicted.models_[1], dt, RelativeMotionModel::CONSTANT_ACCELERATION);
  predicted.predict(predicted.models_[2], dt, RelativeMotionModel::COORDINATED_TURN);
  predicted.timestamp_seconds_ = timestamp_seconds;
  predicted.combine();
  return predicted.combined_;
}

bool RelativeImmEkf::valid(
  double current_time_seconds, double max_data_age_s, double max_position_variance) const
{
  if (!initialized_ || current_time_seconds < last_accepted_measurement_seconds_ ||
    current_time_seconds - last_accepted_measurement_seconds_ > max_data_age_s)
  {
    return false;
  }
  double maximum_variance = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    maximum_variance = std::max(maximum_variance, combined_.covariance[6 * axis + axis]);
  }
  return maximum_variance <= max_position_variance;
}

std::array<double, 3> RelativeImmEkf::model_probabilities() const {return probabilities_;}

}  // namespace px4_offboard_tracking
