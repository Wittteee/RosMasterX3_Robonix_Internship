#pragma once

#include <geometry_msgs/Point.h>
#include <ros/time.h>

#include <cstddef>
#include <deque>
#include <string>

namespace x3_target_navigation {

struct TargetObservation {
  ros::Time stamp;
  std::string frame_id;
  std::string label;
  geometry_msgs::Point point;
  double confidence = 0.0;
  double depth_m = 0.0;
  double valid_ratio = 0.0;
};

struct FusedTarget {
  ros::Time stamp;
  std::string frame_id;
  std::string label;
  geometry_msgs::Point point;
  double covariance[9] = {0.0};
  double confidence = 0.0;
  double depth_m = 0.0;
  double valid_ratio = 0.0;
  std::size_t sample_count = 0;
  bool stable = false;
};

class TargetFusion {
 public:
  TargetFusion(std::size_t min_samples, std::size_t max_samples,
               double max_age_sec, double outlier_distance_m,
               double max_stddev_m);

  bool add(const TargetObservation& observation, const ros::Time& now);
  FusedTarget estimate(const ros::Time& now) const;
  void clear();

 private:
  void prune(const ros::Time& now);
  static double distance(const geometry_msgs::Point& a,
                         const geometry_msgs::Point& b);

  std::size_t min_samples_;
  std::size_t max_samples_;
  double max_age_sec_;
  double outlier_distance_m_;
  double max_stddev_m_;
  std::deque<TargetObservation> observations_;
};

}  // namespace x3_target_navigation
