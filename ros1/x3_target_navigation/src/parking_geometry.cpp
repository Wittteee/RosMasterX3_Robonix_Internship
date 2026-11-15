#include "x3_target_navigation/parking_geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace x3_target_navigation {

ParkingGeometry computeParkingGeometry(const geometry_msgs::Point& robot,
                                       const geometry_msgs::Point& target,
                                       double standoff_m,
                                       double min_travel_m,
                                       double max_travel_m) {
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double target_distance = std::hypot(dx, dy);
  const double travel = target_distance - standoff_m;
  if (!(std::isfinite(robot.x) && std::isfinite(robot.y) &&
        std::isfinite(target.x) && std::isfinite(target.y) &&
        std::isfinite(standoff_m) && std::isfinite(min_travel_m) &&
        std::isfinite(max_travel_m) && standoff_m > 0.0 &&
        min_travel_m >= 0.0 && max_travel_m >= min_travel_m &&
        target_distance > standoff_m && travel >= min_travel_m &&
        travel <= max_travel_m)) {
    throw std::invalid_argument(
        "parking travel outside configured short-navigation range");
  }
  ParkingGeometry result;
  result.travel_m = travel;
  result.position.x = robot.x + travel * dx / target_distance;
  result.position.y = robot.y + travel * dy / target_distance;
  result.position.z = 0.0;
  result.yaw = std::atan2(target.y - result.position.y,
                          target.x - result.position.x);
  return result;
}

}  // namespace x3_target_navigation
