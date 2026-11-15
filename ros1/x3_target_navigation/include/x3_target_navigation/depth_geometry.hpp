#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/CameraInfo.h>

#include <string>

namespace x3_target_navigation {

struct DepthSample {
  double metres = 0.0;
  double valid_ratio = 0.0;
  bool valid = false;
};

DepthSample robustDepth(const cv::Mat& depth, const cv::Rect& detection,
                        double min_depth_m, double max_depth_m,
                        double roi_scale, double min_valid_ratio);

cv::Point3d deproject(double u, double v, double depth_m,
                      const sensor_msgs::CameraInfo& info);

bool finitePoint(const cv::Point3d& point);

}  // namespace x3_target_navigation
