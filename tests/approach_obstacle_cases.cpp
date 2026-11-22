#include "x3_target_navigation/approach_obstacle.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  using x3_target_navigation::blocksApproach;
  const auto blocked = [](double x, double y, double gx = .6, double gy = 0.) {
    return blocksApproach(x, y, gx, gy, .1, .117, .1, .03);
  };
  assert(blocked(.4, 0.));                    // Wall inserted across the route.
  assert(blocked(.4, .19));                   // Body width, not just centre ray.
  assert(!blocked(.4, .21));                  // Side object outside corridor.
  assert(!blocked(.897, 0.));                 // Bottle beyond goal + stop budget.
  assert(!blocked(1.2, 0.));                  // Wall beyond the bottle.
  assert(!blocked(-.3, 0.));                  // Obstacle behind the car.
  assert(blocked(.02, 0.));                   // Already inside front footprint.
  assert(blocked(0., .4, 0., .6));            // Goal rotated 90 degrees.
  assert(!blocked(.4, 0., 0., .6));
  const double c = std::sqrt(.5);
  assert(blocked(.4*c, .4*c, .6*c, .6*c));    // Diagonal corridor.
  assert(!blocked(std::numeric_limits<double>::quiet_NaN(), 0.));
  // The original laser is reversed and displaced 4.35 cm in front of base.
  const double laser_x = -.4;
  const double base_x = .0435 - laser_x;
  assert(blocked(base_x, 0.));
  assert(blocked(.24, 0., 0., 0.));           // At-goal front stopping envelope.
  assert(!blocked(.297, 0., 0., 0.));          // Bottle still outside that envelope.

  using x3_target_navigation::obstacleStopRequired;
  using x3_target_navigation::approachVelocity;
  const double front = .117, threshold = .05;
  assert(!obstacleStopRequired(front + .40, front, threshold));
  assert(!obstacleStopRequired(front + .051, front, threshold));
  assert(obstacleStopRequired(front + .050, front, threshold));
  assert(obstacleStopRequired(front + .049, front, threshold));
  assert(obstacleStopRequired(front - .01, front, threshold));
  assert(!obstacleStopRequired(-.3, front, threshold));
  assert(!obstacleStopRequired(std::numeric_limits<double>::quiet_NaN(), front, threshold));
  // The radial ray can exceed 16.7 cm while its front-plane gap is only 5 cm.
  assert(std::hypot(front + .05, .09) > front + .05);
  assert(obstacleStopRequired(front + .05, front, threshold));

  const double inf = std::numeric_limits<double>::infinity();
  using x3_target_navigation::interveningObstacleGap;
  assert(std::isinf(interveningObstacleGap(-.093, .084, 1., 0., .1, front, .1, .03)));
  assert(std::isfinite(interveningObstacleGap(.10, .05, 1., 0., .1, front, .1, .03)));
  assert(std::abs(interveningObstacleGap(.167, .09, 1., 0., .1, front, .1, .03) - .05) < 1e-9);
  assert(std::isinf(interveningObstacleGap(.167, .21, 1., 0., .1, front, .1, .03)));
  assert(std::isinf(interveningObstacleGap(1.2, 0., 1., 0., .1, front, .1, .03)));
  assert(std::isinf(interveningObstacleGap(.99, 0., 1., 0., .1, front, .1, .03)));
  assert(std::isfinite(interveningObstacleGap(.90, 0., 1., 0., .1, front, .1, .03)));
  assert(approachVelocity(.6, 0., 0., inf, threshold, .04).linear == .04);
  assert(approachVelocity(.6, 0., 0., .4, threshold, .04).linear == .04);
  assert(approachVelocity(.3, 0., 0., .051, threshold, .04).linear == .015);
  assert(approachVelocity(.3, 0., 0., .05, threshold, .04).linear == 0.);
  assert(approachVelocity(.3, 0., .1, .049, threshold, .04).angular == 0.);
  // A nominal goal or its tolerance cannot cut an obstacle approach short.
  assert(approachVelocity(-.01, 0., 0., .07, threshold, .04).linear > 0.);
  assert(!approachVelocity(-.01, 0., 0., .07, threshold, .04).arrived);
  assert(approachVelocity(0., 0., 0., inf, threshold, .04).arrived);
  assert(approachVelocity(.3, .11, 0., inf, threshold, .04).off_route);
  assert(approachVelocity(.3, 0., .4, inf, threshold, .04).off_route);
  // A few centimetres of heading error nudges, and never freezes the approach.
  const double cross_yaw = std::atan2(.03, .30);
  const auto yawing = approachVelocity(.6, -.03, cross_yaw, .40, threshold, .04);
  assert(std::abs(yawing.angular - .05) < 1e-12 && yawing.linear == .04);
  const auto nudge = approachVelocity(.6, 0., .02, .40, threshold, .04);
  assert(std::abs(nudge.angular - .012) < 1e-12 && nudge.linear == .04);
  const auto saturated = approachVelocity(.6, 0., .20, .40, threshold, .04);
  assert(std::abs(saturated.angular - .05) < 1e-12 && saturated.linear == .04);
  // Function 2's straight leg holds heading until the 10 cm bypass trigger.
  using x3_target_navigation::bypassStraightVelocity;
  const auto held = bypassStraightVelocity(.6, -.03, cross_yaw, .40, .10, .04);
  assert(held.angular == 0. && held.linear == .04 && !held.off_route);
  const auto near_trigger = bypassStraightVelocity(.6, 0., 0., .20, .10, .04);
  assert(near_trigger.linear == .015 && near_trigger.angular == 0.);
  const auto at_trigger = bypassStraightVelocity(.6, 0., .10, .10, .10, .04);
  assert(at_trigger.linear == 0. && at_trigger.angular == 0.);
  assert(bypassStraightVelocity(.6, .11, 0., .40, .10, .04).off_route);
  assert(bypassStraightVelocity(.6, 0., .36, .40, .10, .04).off_route);

  // Simulate a wall appearing after the robot starts. On every sample above
  // 5 cm it keeps advancing; the first sample at/below 5 cm commands zero.
  double position = 0.;
  bool stopped = false;
  for (int step = 0; step < 2000; ++step) {
    const double gap = step < 20 ? inf : interveningObstacleGap(
        .50 - position, 0., 1.0 - position, 0., .1, front, .1, .03);
    const auto command = approachVelocity(.70 - position, 0., 0., gap, threshold, .04);
    if (gap <= threshold + 1e-9) {
      assert(command.linear == 0. && command.angular == 0.);
      assert(gap > .049);  // 50 ms sampling, 1.5 cm/s creep.
      stopped = true;
      break;
    }
    assert(command.linear > 0. && !command.arrived);
    position += command.linear * .05;
  }
  assert(stopped);
}
