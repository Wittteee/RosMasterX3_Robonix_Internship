#include "x3_target_navigation/depth_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace x3_target_navigation {
namespace {

double depthAt(const cv::Mat& depth, int row, int col) {
  if (depth.type() == CV_16UC1) {
    return static_cast<double>(depth.at<uint16_t>(row, col)) * 0.001;
  }
  if (depth.type() == CV_32FC1) {
    return static_cast<double>(depth.at<float>(row, col));
  }
  throw std::invalid_argument("depth encoding must be 16UC1 or 32FC1");
}

}  // namespace

DepthSample robustDepth(const cv::Mat& depth, const cv::Rect& detection,
                        double min_depth_m, double max_depth_m,
                        double roi_scale, double min_valid_ratio) {
  if (depth.empty() || depth.channels() != 1) {
    throw std::invalid_argument("depth image is empty or not single-channel");
  }
  if (!(std::isfinite(min_depth_m) && std::isfinite(max_depth_m) &&
        min_depth_m > 0.0 && max_depth_m > min_depth_m)) {
    throw std::invalid_argument("invalid depth range");
  }
  if (!(roi_scale > 0.0 && roi_scale <= 1.0 &&
        min_valid_ratio > 0.0 && min_valid_ratio <= 1.0)) {
    throw std::invalid_argument("invalid depth ROI policy");
  }

  const cv::Rect image_bounds(0, 0, depth.cols, depth.rows);
  const cv::Rect clipped = detection & image_bounds;
  if (clipped.width < 2 || clipped.height < 2) {
    return {};
  }
  const int roi_width = std::max(2, static_cast<int>(clipped.width * roi_scale));
  const int roi_height = std::max(2, static_cast<int>(clipped.height * roi_scale));
  const cv::Rect centre(clipped.x + (clipped.width - roi_width) / 2,
                        clipped.y + (clipped.height - roi_height) / 2,
                        roi_width, roi_height);

  std::vector<double> values;
  values.reserve(static_cast<size_t>(centre.area()));
  for (int row = centre.y; row < centre.y + centre.height; ++row) {
    for (int col = centre.x; col < centre.x + centre.width; ++col) {
      const double value = depthAt(depth, row, col);
      if (std::isfinite(value) && value >= min_depth_m && value <= max_depth_m) {
        values.push_back(value);
      }
    }
  }

  DepthSample result;
  result.valid_ratio = static_cast<double>(values.size()) /
                       static_cast<double>(centre.area());
  if (!values.empty() && result.valid_ratio >= min_valid_ratio) {
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    result.metres = values[middle];
    if (values.size() % 2 == 0) {
      const double lower =
          *std::max_element(values.begin(), values.begin() + middle);
      result.metres = (lower + result.metres) * 0.5;
    }
    result.valid = std::isfinite(result.metres);
    result.nearest_metres = *std::min_element(values.begin(), values.end());
    result.nearest_count = static_cast<std::size_t>(std::count_if(
        values.begin(), values.end(), [&](double value) {
          return value <= result.nearest_metres + 0.08;
        }));
    result.nearest_valid = result.valid && result.nearest_count >= 8;
    // A small foreground cluster can be the partially occluding obstacle.
    // Keep it as nearest surface for parking, but exclude it from the bottle
    // centre depth when it is only a minority of the ROI.
    if (result.nearest_count >= 8 &&
        result.nearest_count * 4U < values.size()) {
      std::vector<double> object_values;
      object_values.reserve(values.size() - result.nearest_count);
      for (const double value : values) {
        if (value > result.nearest_metres + 0.08) {
          object_values.push_back(value);
        }
      }
      if (!object_values.empty()) {
        const size_t object_middle = object_values.size() / 2;
        std::nth_element(object_values.begin(),
                         object_values.begin() + object_middle,
                         object_values.end());
        result.metres = object_values[object_middle];
      }
    }
    for (int row = centre.y; row < centre.y + centre.height; ++row) {
      for (int col = centre.x; col < centre.x + centre.width; ++col) {
        const double value = depthAt(depth, row, col);
        if (std::isfinite(value) &&
            std::abs(value - result.nearest_metres) < 1e-9) {
          result.nearest_u = col;
          result.nearest_v = row;
          return result;
        }
      }
    }
    return result;
  }

  // A glossy or pale object can leave the centre empty while its rim still
  // returns a tight near-range cluster. Use that cluster instead of the wall
  // visible through the hole.
  std::vector<double> box_values;
  box_values.reserve(static_cast<size_t>(clipped.area()));
  for (int row = clipped.y; row < clipped.y + clipped.height; ++row) {
    for (int col = clipped.x; col < clipped.x + clipped.width; ++col) {
      const double value = depthAt(depth, row, col);
      if (std::isfinite(value) && value >= min_depth_m && value <= max_depth_m) {
        box_values.push_back(value);
      }
    }
  }
  constexpr size_t kMinCluster = 80;
  constexpr double kClusterBandM = 0.12;
  if (box_values.size() < kMinCluster) {
    return result;
  }
  std::sort(box_values.begin(), box_values.end());
  size_t cluster_begin = box_values.size();
  for (size_t i = 0; i + kMinCluster <= box_values.size(); ++i) {
    if (box_values[i + kMinCluster - 1] - box_values[i] <= kClusterBandM) {
      cluster_begin = i;
      break;
    }
  }
  if (cluster_begin == box_values.size()) {
    return result;
  }
  size_t cluster_end = cluster_begin + kMinCluster - 1;
  while (cluster_end + 1 < box_values.size() &&
         box_values[cluster_end + 1] - box_values[cluster_begin] <=
             kClusterBandM) {
    ++cluster_end;
  }
  const size_t cluster_size = cluster_end - cluster_begin + 1;
  const size_t cluster_middle = cluster_begin + cluster_size / 2;
  result.metres = box_values[cluster_middle];
  result.nearest_metres = box_values[cluster_begin];
  result.nearest_count = cluster_size;
  result.valid_ratio =
      static_cast<double>(cluster_size) / static_cast<double>(clipped.area());
  result.valid = std::isfinite(result.metres);
  result.nearest_valid = std::isfinite(result.nearest_metres);
  for (int row = clipped.y; row < clipped.y + clipped.height; ++row) {
    for (int col = clipped.x; col < clipped.x + clipped.width; ++col) {
      const double value = depthAt(depth, row, col);
      if (std::isfinite(value) &&
          std::abs(value - result.nearest_metres) < 1e-9) {
        result.nearest_u = col;
        result.nearest_v = row;
        return result;
      }
    }
  }
  return result;
}

cv::Point3d deproject(double u, double v, double depth_m,
                      const sensor_msgs::CameraInfo& info) {
  const double fx = info.K[0];
  const double fy = info.K[4];
  const double cx = info.K[2];
  const double cy = info.K[5];
  if (!(std::isfinite(u) && std::isfinite(v) && std::isfinite(depth_m) &&
        depth_m > 0.0 && std::isfinite(fx) && std::isfinite(fy) &&
        fx > 0.0 && fy > 0.0 && info.width > 0 && info.height > 0 &&
        u >= 0.0 && v >= 0.0 && u < info.width && v < info.height)) {
    throw std::invalid_argument("invalid camera projection input");
  }
  return cv::Point3d((u - cx) * depth_m / fx,
                     (v - cy) * depth_m / fy,
                     depth_m);
}

bool finitePoint(const cv::Point3d& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

}  // namespace x3_target_navigation
