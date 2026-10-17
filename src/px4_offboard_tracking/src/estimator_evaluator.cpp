#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "px4_offboard_tracking/estimator_evaluation.hpp"
#include "px4_offboard_tracking/msg/relative_estimate.hpp"

namespace px4_offboard_tracking
{

// Evaluation-only sink. Truth is consumed here and is never republished to the
// estimator, mission manager, guidance or control path.
class EstimatorEvaluatorNode : public rclcpp::Node
{
public:
  EstimatorEvaluatorNode()
  : Node("estimator_evaluator"),
    truth_topic_(declare_parameter<std::string>(
      "truth_topic", "/evaluation/relative_truth")),
    estimate_topic_(declare_parameter<std::string>(
      "estimate_topic", "/tracking/relative/estimate")),
    truth_history_s_(declare_parameter<double>("truth_history_s", 2.0)),
    synchronization_tolerance_s_(
      declare_parameter<double>("synchronization_tolerance_s", 0.03)),
    report_period_s_(declare_parameter<double>("report_period_s", 5.0)),
    truth_buffer_(truth_history_s_, synchronization_tolerance_s_)
  {
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_topic_, 20,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        truth_buffer_.add({
          rclcpp::Time(msg->header.stamp).seconds(),
          {msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z},
          {msg->twist.twist.linear.x, msg->twist.twist.linear.y,
            msg->twist.twist.linear.z}});
      });
    estimate_sub_ = create_subscription<msg::RelativeEstimate>(
      estimate_topic_, 20,
      [this](msg::RelativeEstimate::UniquePtr estimate) {on_estimate(*estimate);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period_s_), [this]() {report();});
  }

private:
  void on_estimate(const msg::RelativeEstimate & estimate)
  {
    if (!estimate.valid) {
      return;
    }
    const rclcpp::Time estimate_time(estimate.header.stamp);
    RelativeTruthSample truth;
    if (!truth_buffer_.sample(estimate_time.seconds(), truth)) {
      return;
    }
    evaluation_.add({
      {estimate.position.x, estimate.position.y, estimate.position.z},
      {estimate.velocity.x, estimate.velocity.y, estimate.velocity.z},
      truth.position, truth.velocity,
      estimate.innovation, estimate.normalized_innovation_squared,
      std::max(0.0, estimate.coast_time_s)});
  }

  void report()
  {
    const auto metrics = evaluation_.summary();
    if (metrics.sample_count == 0) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "IMM evaluation n=%zu position_rmse=%.3f velocity_rmse=%.3f "
      "innovation_bias=[%.3f %.3f %.3f] mean_nis=%.3f max_coast=%.3f",
      metrics.sample_count, metrics.position_rmse_norm, metrics.velocity_rmse_norm,
      metrics.innovation_bias[0], metrics.innovation_bias[1],
      metrics.innovation_bias[2], metrics.mean_normalized_innovation_squared,
      metrics.maximum_coast_time_s);
  }

  std::string truth_topic_;
  std::string estimate_topic_;
  double truth_history_s_;
  double synchronization_tolerance_s_;
  double report_period_s_;
  EstimatorEvaluation evaluation_;
  RelativeTruthBuffer truth_buffer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<msg::RelativeEstimate>::SharedPtr estimate_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::EstimatorEvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}
