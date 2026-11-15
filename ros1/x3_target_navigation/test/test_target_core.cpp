#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>

#include <stdexcept>

#include "x3_target_navigation/depth_geometry.hpp"
#include "x3_target_navigation/parking_geometry.hpp"
#include "x3_target_navigation/target_fusion.hpp"

namespace x3_target_navigation {
namespace {

TEST(DepthGeometry, UsesMedianAndRejectsSparseDepth) {
  cv::Mat depth(20, 20, CV_16UC1, cv::Scalar(1000));
  depth.at<uint16_t>(10, 10) = 0;
  const DepthSample sample =
      robustDepth(depth, cv::Rect(5, 5, 10, 10), 0.2, 2.0, 0.5, 0.5);
  EXPECT_TRUE(sample.valid);
  EXPECT_NEAR(sample.metres, 1.0, 1e-9);
  EXPECT_GT(sample.valid_ratio, 0.8);

  cv::Mat empty_depth(20, 20, CV_16UC1, cv::Scalar(0));
  EXPECT_FALSE(robustDepth(empty_depth, cv::Rect(5, 5, 10, 10),
                          0.2, 2.0, 0.5, 0.5).valid);
}

TEST(DepthGeometry, SupportsFloatAndDeprojectsPinhole) {
  cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(1.5F));
  EXPECT_NEAR(robustDepth(depth, cv::Rect(0, 0, 10, 10),
                          0.2, 2.0, 1.0, 1.0).metres,
              1.5, 1e-6);
  sensor_msgs::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.K[0] = 500.0;
  info.K[2] = 320.0;
  info.K[4] = 500.0;
  info.K[5] = 240.0;
  info.K[8] = 1.0;
  const cv::Point3d point = deproject(420.0, 190.0, 2.0, info);
  EXPECT_NEAR(point.x, 0.4, 1e-9);
  EXPECT_NEAR(point.y, -0.2, 1e-9);
  EXPECT_NEAR(point.z, 2.0, 1e-9);
}

TargetObservation observation(double stamp, double x, double y) {
  TargetObservation item;
  item.stamp = ros::Time(stamp);
  item.frame_id = "map";
  item.label = "bottle";
  item.point.x = x;
  item.point.y = y;
  item.point.z = 0.5;
  item.confidence = 0.9;
  item.depth_m = 1.0;
  item.valid_ratio = 0.8;
  return item;
}

TEST(TargetFusion, RequiresStableConsecutiveSamples) {
  TargetFusion fusion(3, 5, 1.0, 0.2, 0.04);
  EXPECT_TRUE(fusion.add(observation(10.0, 1.00, 2.00), ros::Time(10.0)));
  EXPECT_TRUE(fusion.add(observation(10.1, 1.01, 2.00), ros::Time(10.1)));
  EXPECT_FALSE(fusion.estimate(ros::Time(10.1)).stable);
  EXPECT_FALSE(fusion.add(observation(10.2, 2.00, 2.00), ros::Time(10.2)));
  EXPECT_TRUE(fusion.add(observation(10.2, 0.99, 2.01), ros::Time(10.2)));
  const FusedTarget result = fusion.estimate(ros::Time(10.2));
  EXPECT_TRUE(result.stable);
  EXPECT_EQ(result.sample_count, 3U);
  EXPECT_NEAR(result.point.x, 1.0, 0.01);
}

TEST(TargetFusion, ExpiresOldSamples) {
  TargetFusion fusion(2, 4, 0.5, 0.2, 0.04);
  EXPECT_TRUE(fusion.add(observation(1.0, 1.0, 2.0), ros::Time(1.0)));
  EXPECT_TRUE(fusion.add(observation(1.1, 1.0, 2.0), ros::Time(1.1)));
  EXPECT_TRUE(fusion.estimate(ros::Time(1.1)).stable);
  EXPECT_FALSE(fusion.estimate(ros::Time(2.0)).stable);
}

TEST(ParkingGeometry, StopsAtStandoffFacingTarget) {
  geometry_msgs::Point robot;
  robot.x = 1.0;
  robot.y = 2.0;
  geometry_msgs::Point target;
  target.x = 2.0;
  target.y = 2.0;
  const ParkingGeometry result =
      computeParkingGeometry(robot, target, 0.4, 0.1, 0.8);
  EXPECT_NEAR(result.travel_m, 0.6, 1e-9);
  EXPECT_NEAR(result.position.x, 1.6, 1e-9);
  EXPECT_NEAR(result.position.y, 2.0, 1e-9);
  EXPECT_NEAR(result.yaw, 0.0, 1e-9);
}

TEST(ParkingGeometry, RejectsUnsafeTravel) {
  geometry_msgs::Point robot;
  geometry_msgs::Point target;
  target.x = 2.0;
  EXPECT_THROW(computeParkingGeometry(robot, target, 0.4, 0.1, 0.8),
               std::invalid_argument);
  target.x = 0.45;
  EXPECT_THROW(computeParkingGeometry(robot, target, 0.4, 0.1, 0.8),
               std::invalid_argument);
}

}  // namespace
}  // namespace x3_target_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
