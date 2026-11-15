#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "x3_target_navigation/TargetEstimate.h"
#include "x3_target_navigation/parking_geometry.hpp"

namespace x3_target_navigation {

class TargetTaskManager {
 public:
  using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  TargetTaskManager()
      : private_nh_("~"),
        move_base_("/move_base", true) {
    private_nh_.param("execute_enabled", execute_enabled_, false);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param("max_pose_std_m", max_pose_std_m_, 0.15);
    private_nh_.param("target_max_age_sec", target_max_age_sec_, 1.0);
    private_nh_.param("target_loss_timeout_sec", target_loss_timeout_sec_, 5.0);
    private_nh_.param("visual_max_age_sec", visual_max_age_sec_, 10.0);
    private_nh_.param("standoff_m", standoff_m_, 0.45);
    private_nh_.param("standoff_tolerance_m", standoff_tolerance_m_, 0.12);
    private_nh_.param("min_travel_m", min_travel_m_, 0.10);
    private_nh_.param("max_travel_m", max_travel_m_, 0.80);
    private_nh_.param("plan_tolerance_m", plan_tolerance_m_, 0.05);
    private_nh_.param("target_drift_tolerance_m", target_drift_tolerance_m_,
                      0.15);
    private_nh_.param("navigation_timeout_sec", navigation_timeout_sec_, 120.0);
    private_nh_.param("verification_timeout_sec", verification_timeout_sec_,
                      5.0);
    validateParameters();

    target_subscriber_ = nh_.subscribe(
        "fused_target", 5, &TargetTaskManager::targetCallback, this);
    visual_detection_subscriber_ = nh_.subscribe(
        "visual_detection", 5,
        &TargetTaskManager::visualDetectionCallback, this);
    pose_subscriber_ = nh_.subscribe(
        "/amcl_pose", 5, &TargetTaskManager::poseCallback, this);
    preview_publisher_ =
        nh_.advertise<geometry_msgs::PoseStamped>("preview_goal", 1, true);
    state_publisher_ =
        nh_.advertise<std_msgs::String>("state", 1, true);
    confirm_service_ = nh_.advertiseService(
        "confirm", &TargetTaskManager::confirm, this);
    cancel_service_ = nh_.advertiseService(
        "cancel", &TargetTaskManager::cancel, this);
    make_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    timer_ = private_nh_.createTimer(
        ros::Duration(0.1), &TargetTaskManager::timerCallback, this);
    setState("IDLE", "waiting for stable RGB-D target");
  }

 private:
  void validateParameters() const {
    const bool valid =
        max_pose_std_m_ > 0.0 && target_max_age_sec_ > 0.0 &&
        target_loss_timeout_sec_ > 0.0 &&
        visual_max_age_sec_ > 0.0 &&
        standoff_m_ > 0.0 &&
        standoff_tolerance_m_ > 0.0 && min_travel_m_ >= 0.10 &&
        max_travel_m_ >= min_travel_m_ && max_travel_m_ <= 0.80 &&
        plan_tolerance_m_ > 0.0 && target_drift_tolerance_m_ > 0.0 &&
        navigation_timeout_sec_ > 0.0 && verification_timeout_sec_ > 0.0;
    if (!valid) {
      throw std::invalid_argument("invalid target task safety parameters");
    }
  }

  static double planarDistance(const geometry_msgs::Point& a,
                               const geometry_msgs::Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (msg->header.frame_id != map_frame_ || msg->header.stamp.isZero()) {
      have_pose_ = false;
      return;
    }
    const double x_variance = msg->pose.covariance[0];
    const double y_variance = msg->pose.covariance[7];
    if (!std::isfinite(x_variance) || !std::isfinite(y_variance) ||
        x_variance < 0.0 || y_variance < 0.0 ||
        std::sqrt(std::max(x_variance, y_variance)) > max_pose_std_m_) {
      have_pose_ = false;
      setState("IDLE", "AMCL pose uncertainty too high for short navigation");
      return;
    }
    robot_pose_.header = msg->header;
    robot_pose_.pose = msg->pose.pose;
    have_pose_ = true;
  }

  bool fresh(const TargetEstimate& target, const ros::Time& now) const {
    const double age = (now - target.header.stamp).toSec();
    return target.stable && target.header.frame_id == map_frame_ &&
           !target.header.stamp.isZero() && age >= 0.0 &&
           age <= target_max_age_sec_;
  }

  void targetCallback(const TargetEstimate::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (!fresh(*message, now) || !have_pose_) {
      return;
    }
    latest_target_ = *message;
    have_target_ = true;
    if (state_ == "VERIFYING") {
      verifyArrival(*message, now);
      return;
    }
    if (state_ != "IDLE" && state_ != "PREVIEW") {
      return;
    }
    geometry_msgs::PoseStamped candidate;
    std::string reason;
    if (!buildParkingGoal(*message, &candidate, &reason) ||
        !planAvailable(candidate, &reason)) {
      setState("IDLE", reason);
      have_preview_ = false;
      return;
    }
    preview_goal_ = candidate;
    target_at_preview_ = *message;
    have_preview_ = true;
    preview_publisher_.publish(preview_goal_);
    setState("PREVIEW", "goal planned; call /target_navigation/confirm");
  }

  void visualDetectionCallback(const std_msgs::Header::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    const double age = (now - message->stamp).toSec();
    if (state_ != "VERIFYING" || !have_pose_ || message->stamp.isZero() ||
        message->stamp <= verification_started_ || age < 0.0 ||
        age > visual_max_age_sec_) {
      return;
    }
    saw_post_arrival_visual_ = true;
    const double goal_error =
        planarDistance(preview_goal_.pose.position, robot_pose_.pose.position);
    if (goal_error <= standoff_tolerance_m_) {
      setState(
          "SUCCEEDED",
          "move_base parking goal and fresh RGB arrival verified");
    } else {
      ROS_WARN_THROTTLE(
          2.0,
          "Fresh RGB target seen but parking goal error %.3f m is outside "
          "tolerance",
          goal_error);
    }
  }

  bool buildParkingGoal(const TargetEstimate& target,
                        geometry_msgs::PoseStamped* goal,
                        std::string* reason) const {
    ParkingGeometry parking;
    try {
      parking = computeParkingGeometry(
          robot_pose_.pose.position, target.point, standoff_m_,
          min_travel_m_, max_travel_m_);
    } catch (const std::invalid_argument& error) {
      *reason = error.what();
      return false;
    }
    goal->header.stamp = ros::Time::now();
    goal->header.frame_id = map_frame_;
    goal->pose.position = parking.position;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, parking.yaw);
    goal->pose.orientation = tf2::toMsg(orientation);
    return true;
  }

  bool planAvailable(const geometry_msgs::PoseStamped& goal,
                     std::string* reason) {
    if (!make_plan_client_.exists()) {
      *reason = "move_base make_plan service unavailable";
      return false;
    }
    nav_msgs::GetPlan service;
    service.request.start = robot_pose_;
    service.request.start.header.stamp = ros::Time::now();
    service.request.goal = goal;
    service.request.tolerance = plan_tolerance_m_;
    if (!make_plan_client_.call(service) || service.response.plan.poses.empty()) {
      *reason = "move_base found no path to parking goal";
      return false;
    }
    return true;
  }

  bool confirm(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (!execute_enabled_) {
      response.success = false;
      response.message = "execute_enabled is false; preview only";
      return true;
    }
    if (state_ != "PREVIEW" || !have_preview_ || !have_target_ ||
        !fresh(latest_target_, now)) {
      response.success = false;
      response.message = "no fresh prepared target preview";
      return true;
    }
    std::string reason;
    if (!planAvailable(preview_goal_, &reason)) {
      setState("FAILED", reason);
      response.success = false;
      response.message = reason;
      return true;
    }
    if (!move_base_.waitForServer(ros::Duration(2.0))) {
      setState("FAILED", "move_base action unavailable");
      response.success = false;
      response.message = "move_base action unavailable";
      return true;
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = preview_goal_;
    goal.target_pose.header.stamp = now;
    navigation_started_ = now;
    move_base_.sendGoal(
        goal,
        boost::bind(&TargetTaskManager::moveDone, this, _1, _2));
    have_preview_ = false;  // one service call consumes this preview
    setState("NAVIGATING", "one-shot confirmed goal sent");
    response.success = true;
    response.message = "goal accepted; confirmation consumed";
    return true;
  }

  bool cancel(std_srvs::Trigger::Request&,
              std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (state_ == "NAVIGATING") {
      move_base_.cancelGoal();
      setState("PREEMPTED", "cancel requested");
      response.success = true;
      response.message = "move_base cancel requested";
    } else {
      response.success = false;
      response.message = "no active navigation goal";
    }
    return true;
  }

  void moveDone(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr&) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      verification_started_ = ros::Time::now();
      saw_post_arrival_visual_ = false;
      setState("VERIFYING", "move_base succeeded; waiting for fresh target");
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED ||
               state == actionlib::SimpleClientGoalState::RECALLED) {
      setState("PREEMPTED", state.toString());
    } else {
      setState("FAILED", "move_base ended: " + state.toString());
    }
  }

  void verifyArrival(const TargetEstimate& target, const ros::Time& now) {
    if (target.header.stamp <= verification_started_ || !fresh(target, now)) {
      return;
    }
    const double drift = planarDistance(target.point, target_at_preview_.point);
    const double final_distance =
        planarDistance(target.point, robot_pose_.pose.position);
    if (drift <= target_drift_tolerance_m_ &&
        std::abs(final_distance - standoff_m_) <= standoff_tolerance_m_) {
      setState("SUCCEEDED", "move_base and visual arrival verified");
    } else {
      setState("FAILED", "arrival visual verification outside tolerance");
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (state_ == "PREVIEW" &&
        (!have_target_ || !fresh(latest_target_, now))) {
      have_preview_ = false;
      setState("IDLE", "prepared target expired before confirmation");
    } else if (state_ == "NAVIGATING" && have_target_ &&
               (now - latest_target_.header.stamp).toSec() >
                   target_loss_timeout_sec_) {
      move_base_.cancelGoal();
      setState("PREEMPTED", "target lost during navigation; cancel requested");
    } else if (state_ == "NAVIGATING" &&
               (now - navigation_started_).toSec() > navigation_timeout_sec_) {
      move_base_.cancelGoal();
      setState("FAILED", "navigation timeout; cancel requested");
    } else if (state_ == "VERIFYING" &&
               (now - verification_started_).toSec() >
                   verification_timeout_sec_) {
      setState(
          "FAILED",
          saw_post_arrival_visual_
              ? "fresh RGB target seen but parking goal remained outside "
                "tolerance"
              : "no fresh visual arrival verification");
    }
  }

  void setState(const std::string& state, const std::string& detail) {
    if (state_ == state && detail_ == detail) {
      return;
    }
    state_ = state;
    detail_ = detail;
    std_msgs::String message;
    message.data = state + ": " + detail;
    state_publisher_.publish(message);
    ROS_INFO("Target navigation %s", message.data.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber target_subscriber_;
  ros::Subscriber visual_detection_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Publisher preview_publisher_;
  ros::Publisher state_publisher_;
  ros::ServiceServer confirm_service_;
  ros::ServiceServer cancel_service_;
  ros::ServiceClient make_plan_client_;
  ros::Timer timer_;
  MoveBaseClient move_base_;
  std::mutex mutex_;
  std::string map_frame_;
  std::string state_;
  std::string detail_;
  bool execute_enabled_ = false;
  bool have_pose_ = false;
  bool have_target_ = false;
  bool have_preview_ = false;
  bool saw_post_arrival_visual_ = false;
  double target_max_age_sec_ = 1.0;
  double target_loss_timeout_sec_ = 5.0;
  double visual_max_age_sec_ = 10.0;
  double standoff_m_ = 0.45;
  double max_pose_std_m_ = 0.15;
  double standoff_tolerance_m_ = 0.12;
  double min_travel_m_ = 0.10;
  double max_travel_m_ = 0.80;
  double plan_tolerance_m_ = 0.05;
  double target_drift_tolerance_m_ = 0.15;
  double navigation_timeout_sec_ = 120.0;
  double verification_timeout_sec_ = 5.0;
  geometry_msgs::PoseStamped robot_pose_;
  geometry_msgs::PoseStamped preview_goal_;
  TargetEstimate latest_target_;
  TargetEstimate target_at_preview_;
  ros::Time navigation_started_;
  ros::Time verification_started_;
};

}  // namespace x3_target_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_task_manager");
  try {
    x3_target_navigation::TargetTaskManager manager;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 2;
  }
  return 0;
}
