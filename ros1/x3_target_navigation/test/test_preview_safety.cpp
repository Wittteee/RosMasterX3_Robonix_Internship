#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace {

bool velocity_received = false;

void velocityCallback(const geometry_msgs::Twist::ConstPtr&) {
  velocity_received = true;
}

TEST(PreviewSafety, ConfirmationIsRejectedAndNoVelocityIsPublished) {
  ros::NodeHandle node;
  ros::Subscriber velocity =
      node.subscribe("/cmd_vel", 1, velocityCallback);
  ros::Subscriber approach_velocity =
      node.subscribe("/target_navigation/approach_velocity", 1, velocityCallback);
  ros::ServiceClient confirm =
      node.serviceClient<std_srvs::Trigger>("/target_navigation/confirm");
  ASSERT_TRUE(confirm.waitForExistence(ros::Duration(5.0)));
  std_srvs::Trigger request;
  ASSERT_TRUE(confirm.call(request));
  EXPECT_FALSE(request.response.success);
  EXPECT_NE(request.response.message.find("preview only"), std::string::npos);

  const ros::Time deadline = ros::Time::now() + ros::Duration(0.5);
  while (ros::ok() && ros::Time::now() < deadline) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  EXPECT_FALSE(velocity_received);
  (void)velocity;
  (void)approach_velocity;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_preview_safety");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
