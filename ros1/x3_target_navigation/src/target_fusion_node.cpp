#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <stdexcept>

#include "x3_target_navigation/TargetEstimate.h"
#include "x3_target_navigation/target_fusion.hpp"

namespace x3_target_navigation {

class TargetFusionNode {
 public:
  TargetFusionNode() : private_nh_("~") {
    int min_samples = 5;
    int max_samples = 12;
    double max_age_sec = 1.0;
    double outlier_distance_m = 0.15;
    double max_stddev_m = 0.04;
    private_nh_.param("min_samples", min_samples, min_samples);
    private_nh_.param("max_samples", max_samples, max_samples);
    private_nh_.param("max_age_sec", max_age_sec, max_age_sec);
    private_nh_.param("outlier_distance_m", outlier_distance_m,
                      outlier_distance_m);
    private_nh_.param("max_stddev_m", max_stddev_m, max_stddev_m);
    if (min_samples < 2 || max_samples < min_samples) {
      throw std::invalid_argument("invalid fusion sample counts");
    }
    fusion_.reset(new TargetFusion(
        static_cast<std::size_t>(min_samples),
        static_cast<std::size_t>(max_samples),
        max_age_sec, outlier_distance_m, max_stddev_m));
    subscriber_ = nh_.subscribe(
        "raw_target", 20, &TargetFusionNode::callback, this);
    publisher_ = nh_.advertise<TargetEstimate>("fused_target", 5);
    timer_ = nh_.createTimer(
        ros::Duration(0.1), &TargetFusionNode::publish, this);
  }

 private:
  void callback(const TargetEstimate::ConstPtr& message) {
    TargetObservation observation;
    observation.stamp = message->header.stamp;
    observation.frame_id = message->header.frame_id;
    observation.label = message->label;
    observation.point = message->point;
    observation.confidence = message->confidence;
    observation.depth_m = message->depth_m;
    observation.valid_ratio = message->depth_valid_ratio;
    if (!fusion_->add(observation, ros::Time::now())) {
      ROS_WARN_THROTTLE(2.0, "Rejected inconsistent target observation");
    }
  }

  void publish(const ros::TimerEvent&) {
    const FusedTarget estimate = fusion_->estimate(ros::Time::now());
    if (!estimate.stable) {
      return;
    }
    TargetEstimate message;
    message.header.stamp = estimate.stamp;
    message.header.frame_id = estimate.frame_id;
    message.label = estimate.label;
    message.point = estimate.point;
    message.confidence = estimate.confidence;
    message.depth_m = estimate.depth_m;
    message.depth_valid_ratio = estimate.valid_ratio;
    message.sample_count = static_cast<uint32_t>(estimate.sample_count);
    std::copy(std::begin(estimate.covariance), std::end(estimate.covariance),
              message.covariance.begin());
    message.stable = true;
    publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std::unique_ptr<TargetFusion> fusion_;
};

}  // namespace x3_target_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_fusion_node");
  try {
    x3_target_navigation::TargetFusionNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 2;
  }
  return 0;
}
