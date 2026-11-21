#pragma once

#include <cmath>
#include <limits>

namespace x3_target_navigation {

constexpr double kStopComparisonToleranceM = 1e-6;  // LaserScan float32 rounding.

// x is the obstacle's forward coordinate in base_footprint, not the raw
// laser range. The threshold is when to COMMAND a stop, with no added brake
// or inflation margin. A small tolerance only covers floating-point rounding.
inline bool obstacleStopRequired(double x, double front, double stop_distance) {
  return std::isfinite(x) && x >= -front &&
         x - front <= stop_distance + kStopComparisonToleranceM;
}

struct ApproachVelocity {
  double linear = 0.0;
  double angular = 0.0;
  bool arrived = false;
  bool off_route = false;
};

// Follow the originally confirmed line, without replanning around obstacles.
// When a blocker is present, its measured gap takes precedence over the
// nominal bottle parking point, so a goal tolerance cannot stop us early.
inline ApproachVelocity approachVelocity(double remaining, double cross_track,
                                         double heading_error, double gap,
                                         double stop_distance, double speed) {
  ApproachVelocity result;
  if (!std::isfinite(remaining) || !std::isfinite(cross_track) ||
      !std::isfinite(heading_error) || std::abs(cross_track) > 0.10 ||
      std::abs(heading_error) > 0.35 || std::isnan(gap)) {
    result.off_route = true;
    return result;
  }
  if (gap <= stop_distance + kStopComparisonToleranceM) return result;
  if (remaining <= 0.0 && !std::isfinite(gap)) {
    result.arrived = true;
    return result;
  }
  result.angular = std::fmax(-0.15, std::fmin(0.15, 1.5 * heading_error));
  if (std::abs(heading_error) <= 0.12) {
    // Creep for the last 15 cm; never stop early solely for a distant blocker.
    const double distance = std::isfinite(gap) ? gap - stop_distance : remaining;
    result.linear = distance < 0.15 ? std::fmin(speed, 0.015) : speed;
  }
  return result;
}

// All coordinates are in base_footprint. Test the swept body corridor up to
// the confirmed parking goal, including the front overhang and stopping budget.
// Points beyond that corridor (including the bottle) must not trigger a stop.
inline bool blocksApproach(double x, double y, double goal_x, double goal_y,
                           double half_width, double front, double margin,
                           double braking) {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(goal_x) || !std::isfinite(goal_y)) return false;
  const double length = std::hypot(goal_x, goal_y);
  const double ux = length > 1e-6 ? goal_x / length : 1.0;
  const double uy = length > 1e-6 ? goal_y / length : 0.0;
  const double along = x * ux + y * uy;
  const double across = std::abs(-x * uy + y * ux);
  return along >= -front && along <= length + front + margin + braking &&
         across <= half_width + margin;
}

inline double interveningObstacleGap(double x, double y, double bottle_x,
                                     double bottle_y, double half_width,
                                     double front, double side_margin,
                                     double target_depth_error) {
  const double none = std::numeric_limits<double>::infinity();
  const double length = std::hypot(bottle_x, bottle_y);
  // This task advances forward. Rear-half chassis returns are not obstacles
  // in front of the car (real scans can hit the rear body at x=-0.093 m).
  if (!std::isfinite(length) || length <= 1e-6 || x <= 0.0) return none;
  const double along = (x * bottle_x + y * bottle_y) / length;
  if (along > length - target_depth_error ||
      !blocksApproach(x, y, bottle_x, bottle_y, half_width, front,
                      side_margin, 0.0)) return none;
  return x - front;
}

}  // namespace x3_target_navigation
