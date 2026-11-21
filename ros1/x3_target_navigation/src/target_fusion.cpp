#include "x3_target_navigation/target_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace x3_target_navigation {

TargetFusion::TargetFusion(std::size_t min_samples, std::size_t max_samples,
                           double max_age_sec, double outlier_distance_m,
                           double max_stddev_m)
    : min_samples_(min_samples),
      max_samples_(max_samples),
      max_age_sec_(max_age_sec),
      outlier_distance_m_(outlier_distance_m),
      max_stddev_m_(max_stddev_m) {
  if (min_samples_ < 2 || max_samples_ < min_samples_ ||
      !(max_age_sec_ > 0.0) || !(outlier_distance_m_ > 0.0) ||
      !(max_stddev_m_ > 0.0)) {
    throw std::invalid_argument("invalid target fusion policy");
  }
}

double TargetFusion::distance(const geometry_msgs::Point& a,
                              const geometry_msgs::Point& b) {
  return std::sqrt(std::pow(a.x - b.x, 2) +
                   std::pow(a.y - b.y, 2) +
                   std::pow(a.z - b.z, 2));
}

void TargetFusion::prune(const ros::Time& now) {
  while (!observations_.empty()) {
    const double age = (now - observations_.front().stamp).toSec();
    if (age >= 0.0 && age <= max_age_sec_) {
      break;
    }
    observations_.pop_front();
  }
}

bool TargetFusion::add(const TargetObservation& observation,
                       const ros::Time& now) {
  prune(now);
  const double age = (now - observation.stamp).toSec();
  const bool finite = std::isfinite(observation.point.x) &&
                      std::isfinite(observation.point.y) &&
                      std::isfinite(observation.point.z) &&
                      std::isfinite(observation.confidence) &&
                      std::isfinite(observation.depth_m) &&
                      std::isfinite(observation.valid_ratio);
  if (!finite || observation.frame_id.empty() || observation.label.empty() ||
      age < -0.05 || age > max_age_sec_ || observation.confidence <= 0.0 ||
      observation.depth_m <= 0.0 || observation.valid_ratio <= 0.0) {
    return false;
  }
  if (!observations_.empty()) {
    if (observation.frame_id != observations_.back().frame_id ||
        observation.label != observations_.back().label ||
        ((!observation.surface_valid || !observations_.back().surface_valid) &&
         distance(observation.point, observations_.back().point) >
             outlier_distance_m_) ||
        (observation.surface_valid && observations_.back().surface_valid &&
         distance(observation.surface_point,
                  observations_.back().surface_point) >
             outlier_distance_m_)) {
      return false;
    }
  }
  observations_.push_back(observation);
  while (observations_.size() > max_samples_) {
    observations_.pop_front();
  }
  return true;
}

FusedTarget TargetFusion::estimate(const ros::Time& now) const {
  FusedTarget result;
  if (observations_.empty()) {
    return result;
  }
  std::vector<const TargetObservation*> active;
  for (const auto& item : observations_) {
    const double age = (now - item.stamp).toSec();
    if (age >= 0.0 && age <= max_age_sec_) {
      active.push_back(&item);
    }
  }
  if (active.empty()) {
    return result;
  }

  result.stamp = active.back()->stamp;
  result.frame_id = active.back()->frame_id;
  result.label = active.back()->label;
  result.sample_count = active.size();
  result.surface_valid = true;
  for (const auto* item : active) {
    result.surface_valid = result.surface_valid && item->surface_valid;
    result.point.x += item->point.x;
    result.point.y += item->point.y;
    result.point.z += item->point.z;
    result.surface_point.x += item->surface_point.x;
    result.surface_point.y += item->surface_point.y;
    result.surface_point.z += item->surface_point.z;
    result.confidence += item->confidence;
    result.depth_m += item->depth_m;
    result.surface_depth_m += item->surface_depth_m;
    result.valid_ratio += item->valid_ratio;
  }
  const double count = static_cast<double>(active.size());
  result.point.x /= count;
  result.point.y /= count;
  result.point.z /= count;
  result.surface_point.x /= count;
  result.surface_point.y /= count;
  result.surface_point.z /= count;
  result.confidence /= count;
  result.depth_m /= count;
  result.surface_depth_m /= count;
  result.valid_ratio /= count;

  for (const auto* item : active) {
    const geometry_msgs::Point& reference =
        result.surface_valid ? item->surface_point : item->point;
    const geometry_msgs::Point& mean =
        result.surface_valid ? result.surface_point : result.point;
    const double delta[3] = {reference.x - mean.x, reference.y - mean.y,
                             reference.z - mean.z};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        result.covariance[row * 3 + col] += delta[row] * delta[col] / count;
      }
    }
  }
  const double max_stddev =
      std::sqrt(std::max({result.covariance[0],
                          result.covariance[4],
                          result.covariance[8]}));
  result.stable = active.size() >= min_samples_ &&
                  std::isfinite(max_stddev) &&
                  max_stddev <= max_stddev_m_;
  return result;
}

void TargetFusion::clear() {
  observations_.clear();
}

}  // namespace x3_target_navigation
