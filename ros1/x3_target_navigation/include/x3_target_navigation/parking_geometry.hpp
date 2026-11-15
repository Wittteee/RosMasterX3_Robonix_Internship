#pragma once

#include <geometry_msgs/Point.h>

namespace x3_target_navigation {

struct ParkingGeometry {
  geometry_msgs::Point position;
  double yaw = 0.0;
  double travel_m = 0.0;
};

ParkingGeometry computeParkingGeometry(const geometry_msgs::Point& robot,
                                       const geometry_msgs::Point& target,
                                       double standoff_m,
                                       double min_travel_m,
                                       double max_travel_m);

}  // namespace x3_target_navigation
