#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <limits>

#include "x3_target_navigation/TargetEstimate.h"
#include "x3_target_navigation/approach_obstacle.hpp"
#include "x3_target_navigation/bypass_geometry.hpp"
#include "x3_target_navigation/parking_geometry.hpp"

namespace x3_target_navigation {

class TargetTaskManager {
 public:
  using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  TargetTaskManager()
      : private_nh_("~"),
        tf_listener_(tf_buffer_),
        move_base_("/move_base", true) {
    private_nh_.param("execute_enabled", execute_enabled_, false);
    private_nh_.param("obstacle_stop_approach", obstacle_stop_approach_, false);
    private_nh_.param("obstacle_bypass_enabled", obstacle_bypass_enabled_, false);
    private_nh_.param("bypass_clearance_m", bypass_clearance_m_, 0.05);
    private_nh_.param("bypass_detection_distance_m", bypass_detection_distance_m_, 0.05);
    private_nh_.param("bypass_max_shift_m", bypass_max_shift_m_, 0.45);
    private_nh_.param("obstacle_stop_distance_m", obstacle_stop_distance_m_, 0.05);
    private_nh_.param("approach_speed_mps", approach_speed_mps_, 0.04);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("camera_frame", camera_frame_,
                                   "camera_rgb_optical_frame");
    private_nh_.param("max_pose_std_m", max_pose_std_m_, 0.15);
    private_nh_.param("target_max_age_sec", target_max_age_sec_, 1.0);
    private_nh_.param("target_loss_timeout_sec", target_loss_timeout_sec_, 5.0);
    private_nh_.param("parking_clearance_m", parking_clearance_m_, 0.12);
    private_nh_.param("clearance_min_m", clearance_min_m_, 0.10);
    private_nh_.param("clearance_max_m", clearance_max_m_, 0.20);
    private_nh_.param("footprint_front_m", footprint_front_m_, 0.117);
    private_nh_.param("footprint_half_width_m", footprint_half_width_m_, 0.100);
    private_nh_.param("depth_error_m", depth_error_m_, 0.03);
    private_nh_.param("braking_distance_m", braking_distance_m_, 0.03);
    private_nh_.param("obstacle_margin_m", obstacle_margin_m_, 0.10);
    private_nh_.param("obstacle_bearing_half_angle_rad",
                      obstacle_bearing_half_angle_rad_, 0.35);
    private_nh_.param("trans_stopped_vel", trans_stopped_vel_, 0.02);
    private_nh_.param("theta_stopped_vel", theta_stopped_vel_, 0.05);
    private_nh_.param("stop_confirmation_timeout_sec",
                      stop_confirmation_timeout_sec_, 2.0);
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
    private_nh_.param("min_voltage_v", min_voltage_v_, 10.0);
    private_nh_.param("max_voltage_v", max_voltage_v_, 13.5);
    private_nh_.param("min_edition", min_edition_, 1.0);
    private_nh_.param("min_scan_hz", min_scan_hz_, 5.0);
    private_nh_.param("sensor_timeout_sec", sensor_timeout_sec_, 1.0);
    validateParameters();

    target_subscriber_ = nh_.subscribe(
        "fused_target", 5, &TargetTaskManager::targetCallback, this);
    visual_detection_subscriber_ = nh_.subscribe(
        "visual_detection", 5,
        &TargetTaskManager::visualDetectionCallback, this);
    pose_subscriber_ = nh_.subscribe(
        "/amcl_pose", 5, &TargetTaskManager::poseCallback, this);
    std::string voltage_topic;
    std::string edition_topic;
    std::string scan_topic;
    std::string rgb_topic;
    std::string depth_topic;
    private_nh_.param<std::string>("voltage_topic", voltage_topic, "/voltage");
    private_nh_.param<std::string>("edition_topic", edition_topic, "/edition");
    private_nh_.param<std::string>("scan_topic", scan_topic, "/scan");
    private_nh_.param<std::string>("rgb_topic", rgb_topic, "/camera/rgb/image_raw");
    private_nh_.param<std::string>("depth_topic", depth_topic,
                                   "/camera/depth_registered/image_raw");
    voltage_subscriber_ = nh_.subscribe(
        voltage_topic, 5, &TargetTaskManager::voltageCallback, this);
    edition_subscriber_ = nh_.subscribe(
        edition_topic, 5, &TargetTaskManager::editionCallback, this);
    scan_subscriber_ = nh_.subscribe(
        scan_topic, obstacle_bypass_enabled_ ? 1 : 20, &TargetTaskManager::scanCallback, this);
    rgb_subscriber_ = nh_.subscribe(
        rgb_topic, 1, &TargetTaskManager::rgbCallback, this);
    depth_subscriber_ = nh_.subscribe(
        depth_topic, 1, &TargetTaskManager::depthCallback, this);
    odom_subscriber_ = nh_.subscribe(
        "/odom", 20, &TargetTaskManager::odomCallback, this);
    bypass_velocity_subscriber_ = nh_.subscribe(
        "bypass_nav_velocity", 1, &TargetTaskManager::bypassVelocityCallback, this);
    preview_publisher_ =
        nh_.advertise<geometry_msgs::PoseStamped>("preview_goal", 1, true);
    state_publisher_ =
        nh_.advertise<std_msgs::String>("state", 1, true);
    motion_permission_publisher_ =
        nh_.advertise<std_msgs::Header>("motion_permission", 1);
    approach_velocity_publisher_ =
        nh_.advertise<geometry_msgs::Twist>("approach_velocity", 1);
    confirm_service_ = nh_.advertiseService(
        "confirm", &TargetTaskManager::confirm, this);
    cancel_service_ = nh_.advertiseService(
        "cancel", &TargetTaskManager::cancel, this);
    make_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    timer_ = private_nh_.createTimer(
        ros::Duration(0.05), &TargetTaskManager::timerCallback, this);
    setState("IDLE", "waiting for stable RGB-D target");
  }

 private:
  void validateParameters() const {
    const bool valid =
        max_pose_std_m_ > 0.0 && target_max_age_sec_ > 0.0 &&
        obstacle_stop_distance_m_ > 0.0 &&
        approach_speed_mps_ > 0.0 && approach_speed_mps_ <= 0.04 &&
        (!obstacle_bypass_enabled_ || obstacle_stop_approach_) &&
        bypass_clearance_m_ >= 0.03 && bypass_detection_distance_m_ >= 0.05 &&
        bypass_detection_distance_m_ <= 0.50 &&
        bypass_max_shift_m_ >= 0.20 && bypass_max_shift_m_ <= 0.60 &&
        target_loss_timeout_sec_ > 0.0 &&
        parking_clearance_m_ >= clearance_min_m_ &&
        parking_clearance_m_ <= clearance_max_m_ &&
        footprint_front_m_ > 0.0 && footprint_half_width_m_ > 0.0 &&
        depth_error_m_ >= 0.0 &&
        braking_distance_m_ >= 0.0 &&
        obstacle_margin_m_ > 0.0 && obstacle_bearing_half_angle_rad_ > 0.0 &&
        trans_stopped_vel_ >= 0.0 && theta_stopped_vel_ >= 0.0 &&
        stop_confirmation_timeout_sec_ > 0.0 &&
        visual_max_age_sec_ > 0.0 &&
        standoff_m_ > 0.0 &&
        standoff_tolerance_m_ > 0.0 && min_travel_m_ >= 0.10 &&
        max_travel_m_ >= min_travel_m_ &&
        max_travel_m_ <= (obstacle_bypass_enabled_ ? 1.20 : 0.80) &&
        plan_tolerance_m_ > 0.0 && target_drift_tolerance_m_ > 0.0 &&
        navigation_timeout_sec_ > 0.0 && verification_timeout_sec_ > 0.0 &&
        min_voltage_v_ > 0.0 && max_voltage_v_ > min_voltage_v_ &&
        min_edition_ > 0.0 && min_scan_hz_ > 0.0 && sensor_timeout_sec_ > 0.0;
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
      if (state_ == "NAVIGATING") requestSafetyStop("LOCALIZATION_UNAVAILABLE");
      return;
    }
    const double x_variance = msg->pose.covariance[0];
    const double y_variance = msg->pose.covariance[7];
    if (!std::isfinite(x_variance) || !std::isfinite(y_variance) ||
        x_variance < 0.0 || y_variance < 0.0 ||
        std::sqrt(std::max(x_variance, y_variance)) > max_pose_std_m_) {
      have_pose_ = false;
      if (state_ == "NAVIGATING") {
        requestSafetyStop("LOCALIZATION_UNAVAILABLE");
      } else if (!task_locked_ && !stop_waiting_) {
        setState("IDLE", "AMCL pose uncertainty too high for short navigation");
      }
      return;
    }
    robot_pose_.header = msg->header;
    robot_pose_.pose = msg->pose.pose;
    std::copy(msg->pose.covariance.begin(), msg->pose.covariance.end(),
              robot_pose_covariance_);
    have_pose_ = true;
  }

  bool fresh(const TargetEstimate& target, const ros::Time& now) const {
    const double age = (now - target.header.stamp).toSec();
    return target.stable && target.surface_valid &&
           target.header.frame_id == map_frame_ &&
           !target.header.stamp.isZero() && age >= 0.0 &&
           age <= target_max_age_sec_;
  }

  void targetCallback(const TargetEstimate::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (!fresh(*message, now) || !have_pose_) {
      return;
    }
    if (task_locked_ || stop_waiting_) {
      return;
    }
    if (state_ == "NAVIGATING") {
      latest_target_ = *message;
      have_target_ = true;
      return;  // Do not let a sensor callback overwrite an active task state.
    }
    const std::string blocked = precheckReason(now);
    if (!blocked.empty()) {
      have_preview_ = false;
      setState("IDLE", blocked);
      return;
    }
    latest_target_ = *message;
    have_target_ = true;
    if (state_ == "VERIFYING") {
      verifyArrival(*message, now);
      return;
    }
    // A finished attempt must be able to plan again. NAVIGATING stays latched
    // so a new detection cannot replace the goal already sent to move_base.
    if (state_ == "PREEMPTED" || state_ == "FAILED") {
      have_preview_ = false;
      state_ = "IDLE";
    }
    if (state_ != "IDLE" && state_ != "PREVIEW") {
      return;
    }
    geometry_msgs::PoseStamped candidate;
    std::string reason;
    if (!buildParkingGoal(*message, &candidate, &reason) ||
        (!obstacle_stop_approach_ && !planAvailable(candidate, &reason))) {
      if (reason.find("no path") != std::string::npos ||
          reason.find("BLOCKED_NO_SAFE_PATH") != std::string::npos) {
        setState("FAILED", "BLOCKED_NO_SAFE_PATH: " + reason);
      } else {
        setState("IDLE", reason);
      }
      have_preview_ = false;
      return;
    }
    preview_goal_ = candidate;
    target_at_preview_ = *message;
    have_preview_ = true;
    preview_publisher_.publish(preview_goal_);
    setState("PREVIEW", "goal planned; call /target_navigation/confirm");
  }

  bool visualFresh(const ros::Time& now) const {
    if (last_visual_stamp_.isZero()) {
      return false;
    }
    const double age = (now - last_visual_stamp_).toSec();
    return age >= 0.0 && age <= visual_max_age_sec_;
  }

  void visualDetectionCallback(const std_msgs::Header::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    const double age = (now - message->stamp).toSec();
    if (!message->stamp.isZero() && age >= 0.0 && age <= visual_max_age_sec_) {
      last_visual_stamp_ = message->stamp;
    }
    if (state_ != "VERIFYING" || !have_pose_ || message->stamp.isZero() ||
        message->stamp <= verification_started_ || age < 0.0 ||
        age > visual_max_age_sec_) {
      return;
    }
    saw_post_arrival_visual_ = true;
    // RGB-only evidence is not sufficient for success. The fused RGB-D
    // callback must verify the measured surface clearance.
  }

  bool buildParkingGoal(const TargetEstimate& target,
                        geometry_msgs::PoseStamped* goal,
                        std::string* reason) {
    ParkingGeometry parking;
    try {
      if (!target.surface_valid) {
        *reason = "target has no valid nearest-surface depth";
        return false;
      }
      const double required_standoff = footprint_front_m_ +
          parking_clearance_m_ + depth_error_m_ + braking_distance_m_;
      parking = computeParkingGeometry(
          robot_pose_.pose.position,
          obstacle_stop_approach_ ? target.point : target.surface_point, required_standoff,
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
    obstacle_limited_ = false;
    // Function 1 measures the live front gap. Clipping to a distant obstacle
    // here would otherwise stop at the old 10 cm + braking margin.
    if (obstacle_stop_approach_) return true;
    double obstacle_range = std::numeric_limits<double>::infinity();
    if (have_scan_ && obstacleRangeToward(target.surface_point,
                                           &obstacle_range)) {
      const double safe_travel = obstacle_range - footprint_front_m_ -
          obstacle_margin_m_ - braking_distance_m_;
      if (safe_travel < min_travel_m_) {
        *reason = "BLOCKED_NO_SAFE_PATH: obstacle is inside minimum safe stop";
        return false;
      }
      if (parking.travel_m > safe_travel) {
        geometry_msgs::Point robot = robot_pose_.pose.position;
        const double dx = target.surface_point.x - robot.x;
        const double dy = target.surface_point.y - robot.y;
        const double distance = std::hypot(dx, dy);
        const double ratio = safe_travel / distance;
        goal->pose.position.x = robot.x + ratio * dx;
        goal->pose.position.y = robot.y + ratio * dy;
        obstacle_limited_ = true;
        obstacle_range_m_ = obstacle_range;
      }
    }
    return true;
  }

  bool obstacleRangeToward(const geometry_msgs::Point& target,
                           double* range) const {
    if (!have_scan_ || scan_.ranges.empty()) {
      return false;
    }
    geometry_msgs::TransformStamped laser_to_base;
    try {
      laser_to_base = tf_buffer_.lookupTransform(
          "base_footprint", scan_.header.frame_id, scan_.header.stamp);
    } catch (const tf2::TransformException&) {
      return false;  // Confirmation's corridor check will reject missing TF.
    }
    const double dx = target.x - robot_pose_.pose.position.x;
    const double dy = target.y - robot_pose_.pose.position.y;
    const double robot_yaw = tf2::getYaw(robot_pose_.pose.orientation);
    double bearing = std::atan2(dy, dx) - robot_yaw;
    while (bearing > M_PI) bearing -= 2.0 * M_PI;
    while (bearing < -M_PI) bearing += 2.0 * M_PI;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan_.ranges.size(); ++i) {
      const double angle = scan_.angle_min +
          static_cast<double>(i) * scan_.angle_increment;
      const double value = scan_.ranges[i];
      if (!std::isfinite(value) || value < scan_.range_min ||
          value > scan_.range_max) continue;
      geometry_msgs::PointStamped point, base_point;
      point.point.x = value * std::cos(angle);
      point.point.y = value * std::sin(angle);
      tf2::doTransform(point, base_point, laser_to_base);
      double delta = std::atan2(base_point.point.y, base_point.point.x) - bearing;
      while (delta > M_PI) delta -= 2.0 * M_PI;
      while (delta < -M_PI) delta += 2.0 * M_PI;
      if (std::abs(delta) <= obstacle_bearing_half_angle_rad_) {
        nearest = std::min(nearest, std::hypot(base_point.point.x, base_point.point.y));
      }
    }
    if (!std::isfinite(nearest)) return false;
    *range = nearest;
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
    if (task_locked_ || stop_waiting_) {
      response.success = false;
      response.message = "safety stop latched; restart launch before a new attempt";
      return true;
    }
    const std::string blocked = precheckReason(now);
    if (!blocked.empty()) {
      response.success = false;
      response.message = blocked;
      return true;
    }
    if (state_ != "PREVIEW" || !have_preview_ || !have_target_ ||
        !fresh(latest_target_, now)) {
      response.success = false;
      response.message = "no fresh prepared target preview";
      return true;
    }
    std::string reason;
    reason = approachBlockReason(now);
    if (!reason.empty()) {
      task_locked_ = true;
      have_preview_ = false;
      setState("FAILED", reason);
      response.success = false;
      response.message = reason;
      return true;
    }
    if (obstacle_stop_approach_) {
      // Do not block the single ROS callback thread while costmaps start:
      // doing so queues old scans and can falsely trigger SCAN_UNAVAILABLE.
      if (obstacle_bypass_enabled_ && !move_base_.isServerConnected()) {
        response.success = false;
        response.message = "function 2 requires move_base and its live costmaps";
        return true;
      }
      if (!recent(odom_stamp_, now)) {
        response.success = false;
        response.message = "fresh odometry required for straight approach";
        return true;
      }
      approach_start_ = robot_pose_.pose.position;
      approach_yaw_ = std::atan2(preview_goal_.pose.position.y - approach_start_.y,
                                 preview_goal_.pose.position.x - approach_start_.x);
      double error = approach_yaw_ - tf2::getYaw(robot_pose_.pose.orientation);
      error = std::atan2(std::sin(error), std::cos(error));
      if (std::abs(error) > 0.35) {
        response.success = false;
        response.message = "bottle must be in front for straight approach";
        return true;
      }
      navigation_started_ = now;
      have_preview_ = false;
      bypass_phase_ = "NONE";
      bypass_stage_ = -1;
      setState("NAVIGATING", obstacle_bypass_enabled_ ?
          "confirmed bottle goal; approach bottle, brake at 5 cm then validate bypass" :
          "one-shot confirmed straight approach; stop at 5 cm front gap");
      response.success = true;
      response.message = "straight approach accepted; confirmation consumed";
      return true;
    }
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
      if (!obstacle_stop_approach_ || bypassActionActive()) move_base_.cancelGoal();
      stop_waiting_ = true;
      stop_terminal_ = obstacle_stop_approach_ && !bypassActionActive();
      stop_reason_ = "CANCELLED_BY_OPERATOR";
      stop_started_ = ros::Time::now();
      setState("STOPPING", "cancel requested; waiting for terminal and zero velocity");
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
    if (stop_waiting_) {
      stop_terminal_ = true;
      return;
    }
    if (task_locked_) return;  // A late result cannot clear STOP_UNCONFIRMED.
    if (obstacle_bypass_enabled_ && bypassActionActive()) {
      approach_velocity_publisher_.publish(geometry_msgs::Twist());
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        bypass_phase_ = "LEG_STOP";
        bypass_stop_started_ = ros::Time::now();
        bypass_zero_samples_ = 0;
        setState("NAVIGATING", "BYPASS_LEG_STOP: confirming stopped before next segment");
      } else {
        // The action is already terminal. Do not wait for a second callback.
        bypass_phase_ = "NONE";
        requestSafetyStop("BYPASS_ACTION_FAILED: " + state.toString());
        stop_terminal_ = true;
      }
      publishMotionPermission(ros::Time::now());
      return;
    }
    if (obstacle_stop_approach_) return;
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if (obstacle_limited_) {
        stop_waiting_ = true;
        stop_terminal_ = true;
        stop_reason_ = "BLOCKED_NO_SAFE_PATH";
        stop_started_ = ros::Time::now();
        setState("STOPPING", "obstacle-limited goal reached; confirming zero velocity");
        return;
      }
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

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    linear_speed_mps_ = std::hypot(message->twist.twist.linear.x,
                                   message->twist.twist.linear.y);
    angular_speed_rps_ = std::abs(message->twist.twist.angular.z);
    odom_stamp_ = message->header.stamp;
  }

  void verifyArrival(const TargetEstimate& target, const ros::Time& now) {
    if (target.header.stamp <= verification_started_ || !fresh(target, now)) {
      return;
    }
    const double drift = planarDistance(target.surface_point,
                                        target_at_preview_.surface_point);
    const double surface_distance =
        planarDistance(target.surface_point, robot_pose_.pose.position);
    const double clearance = surface_distance - footprint_front_m_;
    const double pose_std = std::sqrt(std::max(
        robot_pose_covariance_[0], robot_pose_covariance_[7]));
    if (drift <= target_drift_tolerance_m_ &&
        clearance >= clearance_min_m_ && clearance <= clearance_max_m_ &&
        pose_std <= parking_clearance_m_) {
      std::ostringstream detail;
      detail << "visual surface clearance verified: " << clearance << " m";
      setState("SUCCEEDED", detail.str());
    } else {
      std::ostringstream detail;
      detail << (clearance < clearance_min_m_ ? "arrival too close" :
                 clearance > clearance_max_m_ ? "arrival too far" :
                 "arrival visual verification outside tolerance")
             << "; clearance=" << clearance << " m";
      setState("FAILED", detail.str());
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (state_ == "NAVIGATING") {
      const std::string obstacle = approachBlockReason(now);
      const std::string sensors = precheckReason(now);
      if (!obstacle.empty() || !sensors.empty()) {
        requestSafetyStop(obstacle.empty() ? sensors : obstacle);
      }
    }
    publishMotionPermission(now);
    if (stop_waiting_) {
      const bool velocity_stopped =
          recent(odom_stamp_, now) && linear_speed_mps_ <= trans_stopped_vel_ &&
          angular_speed_rps_ <= theta_stopped_vel_;
      if (stop_terminal_ && velocity_stopped) {
        const bool failed = safety_stop_latched_ || stop_reason_ == "NAVIGATION_TIMEOUT" ||
                            stop_reason_ == "BLOCKED_NO_SAFE_PATH";
        setState(failed ? "FAILED" : "PREEMPTED", stop_reason_);
        stop_waiting_ = false;
        task_locked_ = safety_stop_latched_;
      } else if ((now - stop_started_).toSec() >
                 stop_confirmation_timeout_sec_) {
        setState("FAILED", "STOP_UNCONFIRMED");
        stop_waiting_ = false;
        task_locked_ = true;
      }
      return;
    }
    // An unconfirmed preview must return to the sensing loop. Leaving the
    // node in FAILED freezes the latched state after the target messages stop.
    if ((state_ == "PREVIEW" &&
         (!have_target_ || !fresh(latest_target_, now))) ||
        (state_ == "FAILED" &&
         detail_ == "prepared target expired before confirmation")) {
      have_preview_ = false;
      have_target_ = false;
      state_ = "IDLE";
      detail_.clear();
    }
    if (state_ == "IDLE" || state_ == "PREVIEW") {
      const std::string blocked = precheckReason(now);
      if (!blocked.empty()) {
        have_preview_ = false;
        setState("IDLE", blocked);
        return;
      }
      if (state_ == "IDLE" && !have_preview_) {
        setState("IDLE", "waiting for stable RGB-D target");
      }
    }
    if (state_ == "NAVIGATING" && have_target_ &&
               (now - latest_target_.header.stamp).toSec() >
                   target_loss_timeout_sec_ &&
               !visualFresh(now) &&
               !(obstacle_bypass_enabled_ && bypass_phase_ != "NONE") &&
               !(obstacle_stop_approach_ && std::isfinite(front_obstacle_gap_m_))) {
      requestSafetyStop("TARGET_LOST");
    } else if (state_ == "NAVIGATING" &&
               (now - navigation_started_).toSec() > navigation_timeout_sec_) {
      requestSafetyStop("NAVIGATION_TIMEOUT");
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
    if (obstacle_stop_approach_ && state_ == "NAVIGATING") runStraightApproach(now);
  }

  void voltageCallback(const std_msgs::Float32::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    voltage_ = message->data;
    voltage_stamp_ = ros::Time::now();
  }

  void editionCallback(const std_msgs::Float32::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    edition_ = message->data;
    edition_stamp_ = ros::Time::now();
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    scan_stamps_.push_back(now);
    scan_ = *message;
    have_scan_ = true;
    while (!scan_stamps_.empty() &&
           (now - scan_stamps_.front()).toSec() > 2.0) {
      scan_stamps_.pop_front();
    }
    if (state_ == "NAVIGATING") {
      const std::string reason = approachBlockReason(now);
      if (!reason.empty()) requestSafetyStop(reason);
    }
  }

  // Caller holds mutex_. Stop is latched even if the obstacle later disappears.
  void requestSafetyStop(const std::string& reason) {
    if (obstacle_stop_approach_ && reason == "BLOCKED_NO_SAFE_PATH") {
      ROS_WARN("Obstacle stop commanded: front gap %.4f m, threshold %.4f m",
               front_obstacle_gap_m_, obstacle_stop_distance_m_);
    }
    safety_stop_latched_ = true;
    task_locked_ = true;
    have_preview_ = false;
    stop_waiting_ = true;
    const bool cancel_bypass = bypassActionActive();
    stop_terminal_ = obstacle_stop_approach_ && !cancel_bypass;
    stop_reason_ = reason;
    stop_started_ = ros::Time::now();
    setState("STOPPING", reason);
    publishMotionPermission(stop_started_);
    if (!obstacle_stop_approach_ || cancel_bypass) move_base_.cancelGoal();
  }

  void runStraightApproach(const ros::Time& now) {
    if (!recent(odom_stamp_, now)) {
      requestSafetyStop("ODOMETRY_UNAVAILABLE");
      return;
    }
    if (obstacle_bypass_enabled_ && runBypassController(now)) return;
    const double ux = std::cos(approach_yaw_), uy = std::sin(approach_yaw_);
    const double dx = robot_pose_.pose.position.x - approach_start_.x;
    const double dy = robot_pose_.pose.position.y - approach_start_.y;
    if (std::hypot(dx, dy) > max_travel_m_) {
      requestSafetyStop("MAX_APPROACH_TRAVEL_REACHED");
      return;
    }
    const double cross = -dx * uy + dy * ux;
    const double remaining = (preview_goal_.pose.position.x - robot_pose_.pose.position.x) * ux +
        (preview_goal_.pose.position.y - robot_pose_.pose.position.y) * uy;
    const double desired_yaw = approach_yaw_ + std::atan2(-cross, 0.30);
    const double delta = desired_yaw - tf2::getYaw(robot_pose_.pose.orientation);
    const auto command = approachVelocity(remaining, cross,
        std::atan2(std::sin(delta), std::cos(delta)),
        front_obstacle_gap_m_, obstacle_stop_distance_m_, approach_speed_mps_);
    if (command.off_route) { requestSafetyStop("APPROACH_OFF_ROUTE"); return; }
    if (command.arrived) {
      verification_started_ = now;
      saw_post_arrival_visual_ = false;
      setState("VERIFYING", "straight approach reached bottle parking point");
      return;
    }
    geometry_msgs::Twist velocity;
    velocity.linear.x = command.linear;
    velocity.angular.z = command.angular;
    approach_velocity_publisher_.publish(velocity);
  }

  bool bypassActionActive() const {
    return obstacle_bypass_enabled_ &&
        (bypass_phase_ == "SHIFT" || bypass_phase_ == "PASS" || bypass_phase_ == "REJOIN");
  }

  void beginBypassStop(const ros::Time& now) {
    bypass_trigger_side_ = bypass_decision_.side;
    bypass_phase_ = "BRAKING";
    bypass_stop_started_ = now;
    bypass_zero_samples_ = 0;
    bypass_last_zero_odom_ = ros::Time();
    approach_velocity_publisher_.publish(geometry_msgs::Twist());
    setState("NAVIGATING", bypass_decision_.side == BypassSide::LEFT ?
        "BRAKING_FOR_LEFT_BYPASS: obstacle intrudes from right" :
        "BRAKING_FOR_RIGHT_BYPASS: obstacle intrudes from left");
  }

  geometry_msgs::PoseStamped bypassPose(double x, double y, double yaw) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  }

  bool checkBypassPlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal, int stage) {
    if (!make_plan_client_.exists()) { ROS_WARN("Bypass planner service unavailable"); return false; }
    nav_msgs::GetPlan request;
    request.request.start = start;
    request.request.start.header.stamp = ros::Time::now();
    request.request.goal = goal;
    request.request.tolerance = std::min(0.02, plan_tolerance_m_);
    if (!make_plan_client_.call(request) || request.response.plan.poses.empty()) {
      ROS_WARN("Bypass stage %d returned no plan", stage); return false;
    }
    std::vector<BypassPoint> path;
    const double c = std::cos(approach_yaw_), s = std::sin(approach_yaw_);
    double length = 0.0;
    for (const auto& pose : request.response.plan.poses) {
      if (pose.header.frame_id != map_frame_ ||
          !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
        ROS_WARN("Bypass stage %d invalid frame/point: %s", stage, pose.header.frame_id.c_str()); return false;
      }
      const double dx = pose.pose.position.x-bypass_origin_.x;
      const double dy = pose.pose.position.y-bypass_origin_.y;
      if (std::abs(-s*dx+c*dy) > bypass_max_shift_m_) {
        ROS_WARN("Bypass stage %d exceeds lateral bound", stage); return false;
      }
      if (!bypassLegAllowed(bypass_side_, stage, c*dx+s*dy, -s*dx+c*dy,
                           0.0, bypass_shift_y_)) {
        ROS_WARN("Bypass stage %d plan leaves corridor at %.3f %.3f", stage, c*dx+s*dy, -s*dx+c*dy);
        return false;
      }
      BypassPoint point{pose.pose.position.x, pose.pose.position.y};
      if (!path.empty()) length += std::hypot(point.x-path.back().x, point.y-path.back().y);
      path.push_back(point);
    }
    if (length > 2.0 ||
        std::hypot(path.front().x-start.pose.position.x, path.front().y-start.pose.position.y) > 0.08 ||
        std::hypot(path.back().x-goal.pose.position.x, path.back().y-goal.pose.position.y) > 0.04) {
      ROS_WARN("Bypass stage %d plan endpoint/length rejected", stage); return false;
    }
    path.insert(path.begin(), BypassPoint{start.pose.position.x,start.pose.position.y});
    const bool body_clear = stage == 0 ?
        bypassTranslationPathClear(path,bypass_points_map_,tf2::getYaw(start.pose.orientation),
                                   footprint_front_m_,footprint_half_width_m_,0.03) :
        bypassPathClear(path,bypass_points_map_,footprint_front_m_,footprint_half_width_m_,0.03);
    if (!body_clear) {
      ROS_WARN("Bypass stage %d swept footprint blocked", stage); return false;
    }
    return true;
  }

  bool prepareBypass() {
    if (!bypass_decision_.blocked || bypass_decision_.side == BypassSide::NONE) return false;
    if (bypass_decision_.side != bypass_trigger_side_) return false;
    bypass_side_ = bypass_decision_.side;
    bypass_origin_ = robot_pose_.pose.position;
    const double c = std::cos(approach_yaw_), s = std::sin(approach_yaw_);
    const double x = bypass_origin_.x, y = bypass_origin_.y;
    const double standoff = footprint_front_m_+parking_clearance_m_+
        depth_error_m_+braking_distance_m_;
    const double target_dx = target_at_preview_.point.x-x;
    const double target_dy = target_at_preview_.point.y-y;
    const double lateral = bypassShiftForTarget(bypass_decision_,
        {c*target_dx+s*target_dy, -s*target_dx+c*target_dy}, standoff, bypass_max_shift_m_);
    if (!std::isfinite(lateral)) return false;
    bypass_shift_y_ = lateral;
    const double forward = bypass_decision_.pass_x;
    bypass_goals_.clear();
    bypass_goals_.push_back(bypassPose(x-s*lateral, y+c*lateral,
                                     tf2::getYaw(robot_pose_.pose.orientation)));
    bypass_goals_.push_back(bypassPose(x+c*forward-s*lateral,
                                      y+s*forward+c*lateral, approach_yaw_));
    try {
      // Re-approach the same frozen bottle from the clear side; do not insist
      // on returning to a parking point that now lies behind the obstacle.
      const auto parking = computeParkingGeometry(bypass_goals_[1].pose.position,
          target_at_preview_.point, standoff, 0.03, 1.2);
      bypass_goals_.push_back(bypassPose(parking.position.x, parking.position.y, parking.yaw));
    } catch (const std::invalid_argument&) { return false; }
    geometry_msgs::PoseStamped start = robot_pose_;
    for (int i = 0; i < 3; ++i) {
      if (!checkBypassPlan(start, bypass_goals_[i], i)) return false;
      start = bypass_goals_[i];
    }
    return true;
  }

  void sendBypassLeg(int stage, const ros::Time& now) {
    bypass_stage_ = stage;
    bypass_phase_ = stage == 0 ? "SHIFT" : stage == 1 ? "PASS" : "REJOIN";
    bypass_command_stamp_ = now;
    bypass_not_before_ = now+ros::Duration(0.1);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = bypass_goals_[stage];
    goal.target_pose.header.stamp = now;
    move_base_.sendGoal(goal, boost::bind(&TargetTaskManager::moveDone, this, _1, _2));
    setState("NAVIGATING", std::string(bypass_side_ == BypassSide::LEFT ? "BYPASS_LEFT_" : "BYPASS_RIGHT_")+
        bypass_phase_);
  }

  bool runBypassController(const ros::Time& now) {
    if (bypass_phase_ == "NONE") return false;
    if (bypassActionActive()) {
      if ((now-bypass_command_stamp_).toSec() > 0.4) {
        approach_velocity_publisher_.publish(geometry_msgs::Twist());
        if ((now-bypass_command_stamp_).toSec() > 3.0) requestSafetyStop("BYPASS_COMMAND_TIMEOUT");
      }
      return true;
    }
    approach_velocity_publisher_.publish(geometry_msgs::Twist());
    if (odom_stamp_ > bypass_stop_started_ && odom_stamp_ > bypass_last_zero_odom_) {
      bypass_last_zero_odom_ = odom_stamp_;
      bypass_zero_samples_ = linear_speed_mps_ <= trans_stopped_vel_ &&
          angular_speed_rps_ <= theta_stopped_vel_ ? bypass_zero_samples_+1 : 0;
    }
    if (bypass_zero_samples_ < 3) {
      if ((now-bypass_stop_started_).toSec() > stop_confirmation_timeout_sec_)
        requestSafetyStop("BYPASS_STOP_UNCONFIRMED");
      return true;
    }
    if (bypass_phase_ == "BRAKING") {
      if (!bypass_decision_.blocked) {
        bypass_phase_ = "NONE";
        setState("NAVIGATING", "obstacle cleared after confirmed stop; resuming approach");
      } else if (!prepareBypass()) {
        requestSafetyStop("BLOCKED_NO_SAFE_BYPASS");
      } else {
        sendBypassLeg(0, ros::Time::now());
      }
    } else if (bypass_phase_ == "LEG_STOP") {
      if (bypass_stage_ == 2) {
        bypass_phase_ = "NONE";
        verification_started_ = now;
        saw_post_arrival_visual_ = false;
        setState("VERIFYING", "bypass complete; waiting for fresh bottle arrival verification");
      } else if (!checkBypassPlan(robot_pose_, bypass_goals_[bypass_stage_+1], bypass_stage_+1)) {
        requestSafetyStop("BLOCKED_BYPASS_ROUTE_CHANGED");
      } else {
        sendBypassLeg(bypass_stage_+1, now);
      }
    }
    return true;
  }

  void bypassVelocityCallback(const geometry_msgs::Twist::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const ros::Time now = ros::Time::now();
    if (!execute_enabled_ || !bypassActionActive() || state_ != "NAVIGATING" ||
        task_locked_ || stop_waiting_ || now < bypass_not_before_) return;
    const double scan_age = (now-bypass_scan_stamp_).toSec();
    if (!recent(bypass_scan_stamp_, now) || scan_age > 0.35 ||
        !precheckReason(now).empty() || !recent(odom_stamp_, now)) {
      requestSafetyStop("BYPASS_SENSOR_UNAVAILABLE");
      return;
    }
    double vx = message->linear.x, vy = message->linear.y, wz = message->angular.z;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      requestSafetyStop("BYPASS_INVALID_VELOCITY"); return;
    }
    const double scale = std::max(1.0, std::hypot(vx, vy)/0.04);
    vx /= scale; vy /= scale;
    wz = std::max(-0.15, std::min(0.15, wz));
    const double delta = tf2::getYaw(robot_pose_.pose.orientation)-approach_yaw_;
    const double lateral = std::sin(delta)*vx+std::cos(delta)*vy;
    if (bypass_stage_ == 0 && static_cast<int>(bypass_side_)*lateral < -0.005) {
      // Some DWA startup samples still refer to its previous scoring grid.
      // Never forward them. Keep stopped until a valid sample arrives; the
      // command watchdog latches a failure if wrong-side commands persist.
      approach_velocity_publisher_.publish(geometry_msgs::Twist());
      ROS_WARN_THROTTLE(1.0, "Rejected planner command toward blocked side");
      return;
    }
    const double dx = robot_pose_.pose.position.x-bypass_origin_.x;
    const double dy = robot_pose_.pose.position.y-bypass_origin_.y;
    if (std::abs(-std::sin(approach_yaw_)*dx+std::cos(approach_yaw_)*dy) > bypass_max_shift_m_) {
      requestSafetyStop("BYPASS_LATERAL_LIMIT"); return;
    }
    if (!bypassLegAllowed(bypass_side_, bypass_stage_,
        std::cos(approach_yaw_)*dx+std::sin(approach_yaw_)*dy,
        -std::sin(approach_yaw_)*dx+std::cos(approach_yaw_)*dy, 0.0, bypass_shift_y_)) {
      requestSafetyStop("BYPASS_OFF_PLANNED_SIDE"); return;
    }
    for (const auto& point : bypass_points_base_) {
      if (insideSweptFootprint(point, vx, vy, wz, footprint_front_m_, footprint_half_width_m_, 0.03)) {
        requestSafetyStop("BYPASS_DYNAMIC_OBSTACLE"); return;
      }
    }
    geometry_msgs::Twist safe;
    safe.linear.x = vx; safe.linear.y = vy; safe.angular.z = wz;
    approach_velocity_publisher_.publish(safe);
    bypass_command_stamp_ = now;
  }

  void publishMotionPermission(const ros::Time& now) {
    std_msgs::Header permission;
    permission.stamp = now;
    permission.frame_id = state_ == "NAVIGATING" && !stop_waiting_ &&
        !task_locked_ && bypass_phase_ != "BRAKING" &&
        bypass_phase_ != "LEG_STOP" ? "ALLOW" : "STOP";
    motion_permission_publisher_.publish(permission);
  }

  std::string approachBlockReason(const ros::Time& now) {
    front_obstacle_gap_m_ = std::numeric_limits<double>::infinity();
    if (!have_scan_ || !recent(scan_.header.stamp, now) ||
        scan_.header.frame_id.empty() || scan_.ranges.empty() ||
        !std::isfinite(scan_.angle_min) ||
        !std::isfinite(scan_.angle_increment) || scan_.angle_increment == 0.0 ||
        !std::isfinite(scan_.range_min) || !std::isfinite(scan_.range_max) ||
        scan_.range_min < 0.0 || scan_.range_max <= scan_.range_min) {
      return "SCAN_UNAVAILABLE";
    }
    geometry_msgs::TransformStamped laser_to_base;
    geometry_msgs::TransformStamped laser_to_map;
    geometry_msgs::PoseStamped goal_in_base;
    try {
      laser_to_base = tf_buffer_.lookupTransform(
          "base_footprint", scan_.header.frame_id, scan_.header.stamp,
          ros::Duration(0.05));
      geometry_msgs::PoseStamped goal = preview_goal_;
      if (obstacle_stop_approach_) goal.pose.position = target_at_preview_.point;
      goal.header.stamp = scan_.header.stamp;
      tf_buffer_.transform(goal, goal_in_base, "base_footprint", ros::Duration(0.05));
      if (obstacle_bypass_enabled_) {
        laser_to_map = tf_buffer_.lookupTransform(map_frame_, scan_.header.frame_id,
                                                 scan_.header.stamp, ros::Duration(0.05));
      }
    } catch (const tf2::TransformException&) {
      return "OBSTACLE_TF_UNAVAILABLE";
    }
    bool usable = false;
    std::vector<BypassPoint> route_points;
    if (obstacle_bypass_enabled_) {
      bypass_points_base_.clear();
      bypass_points_map_.clear();
    }
    const double robot_yaw = tf2::getYaw(robot_pose_.pose.orientation);
    const double route_yaw = state_ == "NAVIGATING" ? approach_yaw_ :
        robot_yaw+std::atan2(goal_in_base.pose.position.y, goal_in_base.pose.position.x);
    const double route_delta = robot_yaw-route_yaw;
    for (std::size_t i = 0; i < scan_.ranges.size(); ++i) {
      const double range = scan_.ranges[i];
      // +inf is a valid no-return ray. NaN, -inf and out-of-range values
      // cannot establish sensor health or a clear route.
      if (std::isinf(range) && range > 0.0) { usable = true; continue; }
      if (!std::isfinite(range) || range < scan_.range_min ||
          range > scan_.range_max) continue;
      usable = true;
      const double angle = scan_.angle_min + i * scan_.angle_increment;
      geometry_msgs::PointStamped point, base_point;
      point.point.x = range * std::cos(angle);
      point.point.y = range * std::sin(angle);
      tf2::doTransform(point, base_point, laser_to_base);
      if (obstacle_bypass_enabled_) {
        const double x = base_point.point.x, y = base_point.point.y;
        // Laser can see its own rear chassis. Mask only the physical body,
        // while preserving all external side and rear obstacles for bypass.
        if (isChassisReturn(x, y, footprint_front_m_, footprint_half_width_m_)) continue;
        bypass_points_base_.push_back({x,y});
        geometry_msgs::PointStamped map_point;
        tf2::doTransform(point, map_point, laser_to_map);
        bypass_points_map_.push_back({map_point.point.x, map_point.point.y});
        route_points.push_back({std::cos(route_delta)*x-std::sin(route_delta)*y,
                                std::sin(route_delta)*x+std::cos(route_delta)*y});
        continue;
      }
      if (obstacle_stop_approach_) {
        const double gap = interveningObstacleGap(base_point.point.x, base_point.point.y,
            goal_in_base.pose.position.x, goal_in_base.pose.position.y,
            footprint_half_width_m_, footprint_front_m_, obstacle_margin_m_, depth_error_m_);
        if (!std::isfinite(gap)) continue;
        // Threshold uses the actual front plane, not radial laser distance.
        front_obstacle_gap_m_ = std::min(front_obstacle_gap_m_, gap);
        if (obstacleStopRequired(base_point.point.x, footprint_front_m_,
                                 obstacle_stop_distance_m_)) {
          return "BLOCKED_NO_SAFE_PATH";
        }
      } else if (blocksApproach(base_point.point.x, base_point.point.y,
                               goal_in_base.pose.position.x, goal_in_base.pose.position.y,
                               footprint_half_width_m_, footprint_front_m_,
                               obstacle_margin_m_, braking_distance_m_)) {
        return "BLOCKED_NO_SAFE_PATH";
      }
    }
    if (usable && obstacle_bypass_enabled_) {
      bypass_scan_stamp_ = scan_.header.stamp;
      bypass_decision_ = classifyBypass(route_points,
          std::hypot(goal_in_base.pose.position.x, goal_in_base.pose.position.y),
          footprint_front_m_, footprint_half_width_m_, bypass_clearance_m_,
          bypass_detection_distance_m_+(bypass_phase_ == "BRAKING" ? 0.03 : 0.0),
          bypass_max_shift_m_, route_delta);
      if (bypass_decision_.blocked) front_obstacle_gap_m_ = 0.0;  // Track frozen occluded target.
      if (state_ == "NAVIGATING" && bypass_phase_ == "NONE" && bypass_decision_.blocked) {
        if (bypass_decision_.side == BypassSide::NONE) return "BLOCKED_FULL_WIDTH_OR_AMBIGUOUS";
        beginBypassStop(now);
      }
    }
    return usable ? "" : "SCAN_UNAVAILABLE";
  }

  void rgbCallback(const sensor_msgs::Image::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (message->width > 0 && message->height > 0 &&
        !message->header.stamp.isZero()) {
      rgb_stamp_ = message->header.stamp;
    }
  }

  void depthCallback(const sensor_msgs::Image::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (message->width > 0 && message->height > 0 &&
        !message->header.stamp.isZero()) {
      depth_stamp_ = message->header.stamp;
    }
  }

  bool recent(const ros::Time& stamp, const ros::Time& now) const {
    if (stamp.isZero()) {
      return false;
    }
    const double age = (now - stamp).toSec();
    return age >= 0.0 && age <= sensor_timeout_sec_;
  }

  // Caller holds mutex_.
  std::string precheckReason(const ros::Time& now) const {
    if (!recent(edition_stamp_, now) || edition_ < min_edition_) {
      return "precheck failed: chassis version unavailable";
    }
    if (!recent(voltage_stamp_, now) || voltage_ < min_voltage_v_ ||
        voltage_ > max_voltage_v_) {
      return "precheck failed: chassis voltage invalid";
    }
    if (scan_stamps_.size() < static_cast<size_t>(std::ceil(min_scan_hz_ * 2.0)) ||
        !recent(scan_stamps_.back(), now)) {
      return "precheck failed: lidar frequency unavailable";
    }
    if (!recent(rgb_stamp_, now)) {
      return "precheck failed: RGB camera has no data";
    }
    if (!recent(depth_stamp_, now)) {
      return "precheck failed: depth camera has no data";
    }
    std::string tf_error;
    if (!tf_buffer_.canTransform(map_frame_, camera_frame_, ros::Time(0),
                                 &tf_error)) {
      return "precheck failed: camera TF is unavailable";
    }
    const double pose_age = (now - robot_pose_.header.stamp).toSec();
    if (!have_pose_ || robot_pose_.header.stamp.isZero() || pose_age < 0.0 ||
        pose_age > 5.0) {
      return "precheck failed: AMCL pose is not initialized";
    }
    return "";
  }

  void setState(const std::string& state, const std::string& detail) {
    if (state_ == state && detail_ == detail) {
      return;
    }
    state_ = state;
    detail_ = detail;
    if (execute_enabled_ && obstacle_stop_approach_ && state_ != "NAVIGATING") {
      approach_velocity_publisher_.publish(geometry_msgs::Twist());
    }
    publishMotionPermission(ros::Time::now());
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
  ros::Subscriber voltage_subscriber_;
  ros::Subscriber edition_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber rgb_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber bypass_velocity_subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher preview_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher motion_permission_publisher_;
  ros::Publisher approach_velocity_publisher_;
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
  bool obstacle_stop_approach_ = false;
  bool obstacle_bypass_enabled_ = false;
  double bypass_clearance_m_ = 0.05;
  double bypass_detection_distance_m_ = 0.05;
  double bypass_max_shift_m_ = 0.45;
  BypassDecision bypass_decision_;
  std::vector<BypassPoint> bypass_points_base_, bypass_points_map_;
  std::string bypass_phase_ = "NONE";
  int bypass_stage_ = -1;
  int bypass_zero_samples_ = 0;
  geometry_msgs::Point bypass_origin_;
  BypassSide bypass_side_ = BypassSide::NONE;
  BypassSide bypass_trigger_side_ = BypassSide::NONE;
  double bypass_shift_y_ = 0.0;
  std::vector<geometry_msgs::PoseStamped> bypass_goals_;
  ros::Time bypass_stop_started_, bypass_last_zero_odom_, bypass_scan_stamp_;
  ros::Time bypass_command_stamp_, bypass_not_before_;
  double obstacle_stop_distance_m_ = 0.05;
  double approach_speed_mps_ = 0.04;
  double front_obstacle_gap_m_ = std::numeric_limits<double>::infinity();
  geometry_msgs::Point approach_start_;
  double approach_yaw_ = 0.0;
  bool have_pose_ = false;
  bool have_target_ = false;
  bool have_preview_ = false;
  bool saw_post_arrival_visual_ = false;
  bool stop_waiting_ = false;
  bool stop_terminal_ = false;
  bool task_locked_ = false;
  bool safety_stop_latched_ = false;
  std::string stop_reason_;
  ros::Time stop_started_;
  double target_max_age_sec_ = 1.0;
  double target_loss_timeout_sec_ = 5.0;
  double parking_clearance_m_ = 0.12;
  double clearance_min_m_ = 0.10;
  double clearance_max_m_ = 0.20;
  double footprint_front_m_ = 0.117;
  double footprint_half_width_m_ = 0.100;
  double depth_error_m_ = 0.03;
  double braking_distance_m_ = 0.03;
  double obstacle_margin_m_ = 0.10;
  double obstacle_bearing_half_angle_rad_ = 0.35;
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
  double min_voltage_v_ = 10.0;
  double max_voltage_v_ = 13.5;
  double min_edition_ = 1.0;
  double min_scan_hz_ = 5.0;
  double sensor_timeout_sec_ = 1.0;
  double voltage_ = 0.0;
  double edition_ = 0.0;
  std::string camera_frame_;
  ros::Time voltage_stamp_;
  ros::Time edition_stamp_;
  ros::Time rgb_stamp_;
  ros::Time depth_stamp_;
  std::deque<ros::Time> scan_stamps_;
  sensor_msgs::LaserScan scan_;
  bool have_scan_ = false;
  bool obstacle_limited_ = false;
  double obstacle_range_m_ = 0.0;
  double trans_stopped_vel_ = 0.02;
  double theta_stopped_vel_ = 0.05;
  double stop_confirmation_timeout_sec_ = 2.0;
  double linear_speed_mps_ = 0.0;
  double angular_speed_rps_ = 0.0;
  ros::Time odom_stamp_;
  geometry_msgs::PoseStamped robot_pose_;
  double robot_pose_covariance_[36] = {0.0};
  geometry_msgs::PoseStamped preview_goal_;
  TargetEstimate latest_target_;
  TargetEstimate target_at_preview_;
  ros::Time navigation_started_;
  ros::Time verification_started_;
  ros::Time last_visual_stamp_;
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
