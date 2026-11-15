#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "x3_target_navigation/TargetEstimate.h"
#include "x3_target_navigation/depth_geometry.hpp"

namespace x3_target_navigation {
namespace {

const std::vector<std::string>& cocoClasses() {
  static const std::vector<std::string> classes = {
      "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
      "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
      "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
      "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
      "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
      "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
      "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard",
      "cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
      "teddy bear","hair drier","toothbrush"};
  return classes;
}

struct Detection {
  cv::Rect box;
  float confidence = 0.0F;
};

class YoloX {
 public:
  YoloX(const std::string& model, const std::string& label,
        float confidence, float nms, bool external_focus)
      : confidence_(confidence), nms_(nms), external_focus_(external_focus) {
    const auto& classes = cocoClasses();
    const auto found = std::find(classes.begin(), classes.end(), label);
    if (found == classes.end()) {
      throw std::invalid_argument("target_label is not a COCO class");
    }
    class_id_ = static_cast<int>(std::distance(classes.begin(), found));
    if (!(confidence_ > 0.0F && confidence_ <= 1.0F &&
          nms_ > 0.0F && nms_ <= 1.0F)) {
      throw std::invalid_argument("invalid YOLOX thresholds");
    }
    net_ = cv::dnn::readNetFromONNX(model);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }

  bool detectUnique(const cv::Mat& bgr, Detection* detection) {
    if (bgr.empty() || bgr.type() != CV_8UC3) {
      throw std::invalid_argument("RGB image must be nonempty BGR8");
    }
    const double scale = std::min(640.0 / bgr.rows, 640.0 / bgr.cols);
    cv::Mat resized;
    cv::resize(bgr, resized,
               cv::Size(std::max(1, static_cast<int>(bgr.cols * scale)),
                        std::max(1, static_cast<int>(bgr.rows * scale))));
    cv::Mat canvas(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(0, 0, resized.cols, resized.rows)));
    cv::Mat blob;
    if (external_focus_) {
      cv::Mat rgb;
      cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
      rgb.convertTo(rgb, CV_32F);
      const int dimensions[4] = {1, 12, 320, 320};
      blob = cv::Mat(4, dimensions, CV_32F);
      float* destination = reinterpret_cast<float*>(blob.data);
      const int patch_y[4] = {0, 1, 0, 1};
      const int patch_x[4] = {0, 0, 1, 1};
      for (int patch = 0; patch < 4; ++patch) {
        for (int channel = 0; channel < 3; ++channel) {
          const size_t plane =
              static_cast<size_t>(patch * 3 + channel) * 320U * 320U;
          for (int y = 0; y < 320; ++y) {
            for (int x = 0; x < 320; ++x) {
              destination[plane + static_cast<size_t>(y) * 320U + x] =
                  rgb.at<cv::Vec3f>(2 * y + patch_y[patch],
                                    2 * x + patch_x[patch])[channel];
            }
          }
        }
      }
    } else {
      blob = cv::dnn::blobFromImage(
          canvas, 1.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
    }
    net_.setInput(blob);
    cv::Mat output = net_.forward();
    if (output.dims != 3 || output.size[0] != 1 ||
        output.size[1] != 8400 || output.size[2] != 85) {
      throw std::runtime_error("unexpected YOLOX output shape");
    }
    cv::Mat rows = output.reshape(1, 8400);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    int index = 0;
    for (const int stride : {8, 16, 32}) {
      const int side = 640 / stride;
      for (int grid_y = 0; grid_y < side; ++grid_y) {
        for (int grid_x = 0; grid_x < side; ++grid_x, ++index) {
          const float* row = rows.ptr<float>(index);
          const float score = row[4] * row[5 + class_id_];
          if (!std::isfinite(score) || score < confidence_) {
            continue;
          }
          const double centre_x = (row[0] + grid_x) * stride / scale;
          const double centre_y = (row[1] + grid_y) * stride / scale;
          const double width = std::exp(row[2]) * stride / scale;
          const double height = std::exp(row[3]) * stride / scale;
          if (!(std::isfinite(width) && std::isfinite(height))) {
            continue;
          }
          const int x1 = std::max(0, static_cast<int>(centre_x - width * 0.5));
          const int y1 = std::max(0, static_cast<int>(centre_y - height * 0.5));
          const int x2 = std::min(bgr.cols, static_cast<int>(centre_x + width * 0.5));
          const int y2 = std::min(bgr.rows, static_cast<int>(centre_y + height * 0.5));
          if (x2 > x1 && y2 > y1) {
            boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
            scores.push_back(score);
          }
        }
      }
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confidence_, nms_, keep);
    if (keep.size() != 1U) {
      ROS_WARN_THROTTLE(2.0, "Expected one target after NMS, found %zu", keep.size());
      return false;
    }
    detection->box = boxes[keep.front()];
    detection->confidence = scores[keep.front()];
    return true;
  }

 private:
  cv::dnn::Net net_;
  int class_id_ = -1;
  float confidence_;
  float nms_;
  bool external_focus_;
};

}  // namespace

class RgbdTargetNode {
 public:
  using Policy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;

  RgbdTargetNode()
      : private_nh_("~"),
        tf_listener_(tf_buffer_) {
    std::string model_path;
    private_nh_.param<std::string>("model_path", model_path, "");
    private_nh_.param<std::string>("target_label", target_label_, "bottle");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param("confidence_threshold", confidence_threshold_, 0.5);
    private_nh_.param("nms_threshold", nms_threshold_, 0.45);
    private_nh_.param("min_depth_m", min_depth_m_, 0.25);
    private_nh_.param("max_depth_m", max_depth_m_, 3.0);
    private_nh_.param("depth_roi_scale", depth_roi_scale_, 0.4);
    private_nh_.param("min_depth_valid_ratio", min_valid_ratio_, 0.5);
    private_nh_.param("max_observation_age_sec", max_age_sec_, 0.5);
    private_nh_.param("tf_timeout_sec", tf_timeout_sec_, 0.2);
    private_nh_.param("external_focus", external_focus_, true);
    if (model_path.empty()) {
      throw std::invalid_argument("~model_path is required");
    }
    detector_.reset(new YoloX(model_path, target_label_,
                              confidence_threshold_, nms_threshold_,
                              external_focus_));

    std::string rgb_topic;
    std::string depth_topic;
    std::string camera_info_topic;
    private_nh_.param<std::string>("rgb_topic", rgb_topic, "/camera/rgb/image_raw");
    private_nh_.param<std::string>("depth_topic", depth_topic,
                                   "/camera/depth_registered/image_raw");
    private_nh_.param<std::string>("camera_info_topic", camera_info_topic,
                                   "/camera/depth_registered/camera_info");
    rgb_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        nh_, rgb_topic, 1));
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        nh_, depth_topic, 1));
    info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(
        nh_, camera_info_topic, 1));
    sync_.reset(new message_filters::Synchronizer<Policy>(
        Policy(8), *rgb_sub_, *depth_sub_, *info_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.08));
    sync_->registerCallback(
        boost::bind(&RgbdTargetNode::callback, this, _1, _2, _3));
    publisher_ = nh_.advertise<TargetEstimate>("raw_target", 5);
    visual_detection_publisher_ =
        nh_.advertise<std_msgs::Header>("visual_detection", 5);
  }

 private:
  static cv::Mat rgbMat(const sensor_msgs::Image& image) {
    if (image.width == 0 || image.height == 0 ||
        image.step < image.width * 3U ||
        image.data.size() < image.step * image.height) {
      throw std::invalid_argument("invalid RGB image buffer");
    }
    cv::Mat view(static_cast<int>(image.height), static_cast<int>(image.width),
                 CV_8UC3, const_cast<uint8_t*>(image.data.data()), image.step);
    if (image.encoding == sensor_msgs::image_encodings::BGR8) {
      return view;
    }
    if (image.encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat converted;
      cv::cvtColor(view, converted, cv::COLOR_RGB2BGR);
      return converted;
    }
    throw std::invalid_argument("RGB encoding must be bgr8 or rgb8");
  }

  static cv::Mat depthMat(const sensor_msgs::Image& image) {
    if (image.is_bigendian) {
      throw std::invalid_argument("big-endian depth images are unsupported");
    }
    int type = -1;
    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      type = CV_16UC1;
    } else if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      type = CV_32FC1;
    } else {
      throw std::invalid_argument("depth encoding must be 16UC1 or 32FC1");
    }
    const size_t pixel_size = type == CV_16UC1 ? sizeof(uint16_t) : sizeof(float);
    if (image.width == 0 || image.height == 0 ||
        image.step < image.width * pixel_size ||
        image.data.size() < image.step * image.height) {
      throw std::invalid_argument("invalid depth image buffer");
    }
    return cv::Mat(static_cast<int>(image.height), static_cast<int>(image.width),
                   type, const_cast<uint8_t*>(image.data.data()), image.step);
  }

  void callback(const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth,
                const sensor_msgs::CameraInfoConstPtr& info) {
    try {
      const double age = (ros::Time::now() - rgb->header.stamp).toSec();
      if (rgb->header.stamp.isZero() || age < -0.05 || age > max_age_sec_) {
        ROS_WARN_THROTTLE(2.0, "Rejecting stale RGB-D observation");
        return;
      }
      if (rgb->header.frame_id.empty() ||
          rgb->header.frame_id != depth->header.frame_id ||
          depth->header.frame_id != info->header.frame_id) {
        ROS_WARN_THROTTLE(
            2.0, "RGB, registered depth and CameraInfo frames differ");
        return;
      }
      const cv::Mat bgr = rgbMat(*rgb);
      const cv::Mat depth_image = depthMat(*depth);
      if (bgr.size() != depth_image.size() ||
          info->width != static_cast<uint32_t>(depth_image.cols) ||
          info->height != static_cast<uint32_t>(depth_image.rows)) {
        ROS_WARN_THROTTLE(2.0, "RGB, registered depth and CameraInfo sizes differ");
        return;
      }
      Detection detection;
      if (!detector_->detectUnique(bgr, &detection)) {
        return;
      }
      // Publish recognition independently of depth. Transparent objects can
      // have no valid close-range depth, but a fresh RGB detection can still
      // close the arrival loop after the RGB-D map goal has been consumed.
      visual_detection_publisher_.publish(rgb->header);
      const DepthSample sample =
          robustDepth(depth_image, detection.box, min_depth_m_, max_depth_m_,
                      depth_roi_scale_, min_valid_ratio_);
      if (!sample.valid) {
        ROS_WARN_THROTTLE(2.0, "Target depth is not sufficiently valid");
        return;
      }
      const double u = detection.box.x + detection.box.width * 0.5;
      const double v = detection.box.y + detection.box.height * 0.5;
      const cv::Point3d camera_point = deproject(u, v, sample.metres, *info);
      geometry_msgs::PointStamped source;
      source.header = depth->header;
      source.point.x = camera_point.x;
      source.point.y = camera_point.y;
      source.point.z = camera_point.z;
      const geometry_msgs::PointStamped mapped =
          tf_buffer_.transform(source, map_frame_, ros::Duration(tf_timeout_sec_));

      TargetEstimate message;
      message.header = mapped.header;
      message.label = target_label_;
      message.point = mapped.point;
      message.confidence = detection.confidence;
      message.depth_m = static_cast<float>(sample.metres);
      message.depth_valid_ratio = static_cast<float>(sample.valid_ratio);
      message.sample_count = 1;
      message.stable = false;
      std::fill(message.covariance.begin(), message.covariance.end(), 0.0);
      publisher_.publish(message);
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Target TF unavailable: %s", error.what());
    } catch (const cv::Exception& error) {
      ROS_ERROR_THROTTLE(2.0, "RGB-D OpenCV error: %s", error.what());
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(2.0, "RGB-D target rejected: %s", error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Publisher visual_detection_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<YoloX> detector_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> rgb_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> info_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::string target_label_;
  std::string map_frame_;
  double confidence_threshold_ = 0.5;
  double nms_threshold_ = 0.45;
  double min_depth_m_ = 0.25;
  double max_depth_m_ = 3.0;
  double depth_roi_scale_ = 0.4;
  double min_valid_ratio_ = 0.5;
  double max_age_sec_ = 0.5;
  double tf_timeout_sec_ = 0.2;
  bool external_focus_ = true;
};

}  // namespace x3_target_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "rgbd_target_node");
  try {
    x3_target_navigation::RgbdTargetNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    std::cerr << "rgbd_target_node startup failed: " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
