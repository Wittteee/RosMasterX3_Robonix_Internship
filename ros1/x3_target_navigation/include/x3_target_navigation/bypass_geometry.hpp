#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace x3_target_navigation {

struct BypassPoint { double x = 0.0; double y = 0.0; };
enum class BypassSide { NONE = 0, LEFT = 1, RIGHT = -1 };

struct BypassDecision {
  bool blocked = false;
  bool full_width = false;
  BypassSide side = BypassSide::NONE;
  double shift_y = 0.0;
  double pass_x = 0.0;
  double front_gap = std::numeric_limits<double>::infinity();
};

// Coordinates are relative to the CURRENT robot, aligned with the confirmed
// approach line. Positive y is left. A clear centre ray is not sufficient:
// both sides of the complete footprint and the clearance margin are checked.
inline BypassDecision classifyBypass(const std::vector<BypassPoint>& points,
                                    double bottle_distance, double front,
                                    double half_width, double clearance,
                                    double lookahead, double max_shift,
                                    double robot_heading_delta = 0.0) {
  BypassDecision result;
  double nearest_x = std::numeric_limits<double>::infinity();
  for (const auto& p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x <= 0.0 ||
        p.x >= bottle_distance-0.03 || std::abs(p.y)>half_width+clearance) continue;
    const double body_x = std::cos(robot_heading_delta)*p.x+
                          std::sin(robot_heading_delta)*p.y;
    if (body_x <= 0.0) continue;
    result.front_gap = std::min(result.front_gap, body_x-front);
    if (body_x <= front+lookahead) nearest_x = std::min(nearest_x, p.x);
  }
  if (!std::isfinite(nearest_x)) return result;
  result.blocked = true;
  double low = std::numeric_limits<double>::infinity();
  double high = -low, far = 0.0;
  for (const auto& p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x <= 0.0 ||
        p.x > nearest_x+0.10 || p.x >= bottle_distance - 0.03 ||
        std::abs(p.y) > half_width + clearance) continue;
    low = std::min(low, p.y);
    high = std::max(high, p.y);
    far = std::max(far, p.x);
  }
  // Trigger uses only the measured front gap. Side classification includes
  // the nearby face beyond that boundary, so a tilted face is not reduced to
  // its first one or two rays with a misleading left/right sign.
  // A wall spanning the footprint is function 1's full-block case. An
  // ambiguous, symmetric central obstruction is not assigned a random side.
  result.full_width = low <= -half_width && high >= half_width;
  const double middle = (low + high) * 0.5;
  if (result.full_width || std::abs(middle) < 0.01) return result;
  result.side = middle < 0.0 ? BypassSide::LEFT : BypassSide::RIGHT;
  const double offset = result.side == BypassSide::LEFT ?
      std::max(0.20, high + half_width + clearance + 0.05) :
      std::min(-0.20, low - half_width - clearance - 0.05);
  if (std::abs(offset) > max_shift) { result.side = BypassSide::NONE; return result; }
  result.shift_y = offset;
  result.pass_x = far + front + clearance + 0.10;
  return result;
}

inline double bypassShiftForTarget(const BypassDecision& decision,
                                   const BypassPoint& target, double standoff,
                                   double max_shift) {
  const double sign = static_cast<int>(decision.side);
  if (!decision.blocked || sign == 0.0 || !std::isfinite(target.x) ||
      !std::isfinite(target.y) || !std::isfinite(standoff) || standoff <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  // A bottle close behind the blocker may be too close to the PASS endpoint
  // after the minimum side step. Leave room to approach it from the clear side
  // rather than moving closer than the required parking standoff.
  const double distance = standoff + 0.08;
  const double dx = target.x-decision.pass_x;
  const double side_distance = std::sqrt(std::max(0.0, distance*distance-dx*dx));
  const double shift = sign*std::max(sign*decision.shift_y, sign*target.y+side_distance);
  return std::abs(shift) <= max_shift ? shift : std::numeric_limits<double>::quiet_NaN();
}

inline bool isChassisReturn(double x, double y, double front, double half_width) {
  if (std::abs(x) <= front && std::abs(y) <= half_width) return true;
  // Measured rear-centre plate return sits on x=-0.117 m. Millimetre range
  // quantization moves it just outside the nominal rectangle. Extend only
  // that narrow rear-centre patch by 3 mm, not the side/front safety region.
  return x >= -front-0.003 && x < -front && std::abs(y) <= 0.025;
}

inline bool insideSweptFootprint(const BypassPoint& p, double vx, double vy,
                                 double wz, double front, double half_width,
                                 double margin, double horizon = 0.5) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) return true;
  // Check a curved holonomic trajectory, including front/rear corners, not
  // just a forward sector. Used as an independent gate on planner commands.
  for (double t = 0.0; t <= horizon + 1e-9; t += 0.05) {
    double x = vx*t, y = vy*t;
    if (std::abs(wz) > 1e-6) {
      x = (vx*std::sin(wz*t) + vy*(std::cos(wz*t)-1.0))/wz;
      y = (vx*(1.0-std::cos(wz*t)) + vy*std::sin(wz*t))/wz;
    }
    const double dx = p.x-x, dy = p.y-y;
    const double rx = std::cos(wz*t)*dx + std::sin(wz*t)*dy;
    const double ry = -std::sin(wz*t)*dx + std::cos(wz*t)*dy;
    if (std::abs(rx) <= front+margin && std::abs(ry) <= half_width+margin) return true;
  }
  return false;
}

inline bool bypassLegAllowed(BypassSide side, int stage, double x, double y,
                              double shift_start_x, double minimum_side_y) {
  const double sign = static_cast<int>(side);
  if (!std::isfinite(x) || !std::isfinite(y) || sign == 0.0) return false;
  // Navfn's 5 cm grid can snap the very first path point one cell behind/to
  // the side of the actual start. Permit only that local start artifact;
  // the entire path still passes the independent swept-footprint test, and
  // the velocity gate never permits motion toward the blocked side.
  const bool snapped_start = stage == 0 && std::hypot(x-shift_start_x,y) <= 0.08;
  if (sign*y < -0.025 && !snapped_start) return false;
  if (stage == 0 && std::abs(x-shift_start_x) > 0.06) return false;
  if (stage == 1 && sign*y < sign*minimum_side_y-0.06) return false;
  return true;
}

inline double pointSegmentDistance(const BypassPoint& p, const BypassPoint& a,
                                    const BypassPoint& b) {
  const double dx = b.x-a.x, dy = b.y-a.y;
  const double length2 = dx*dx+dy*dy;
  const double t = length2 > 1e-12 ? std::max(0.0, std::min(1.0,
      ((p.x-a.x)*dx+(p.y-a.y)*dy)/length2)) : 0.0;
  return std::hypot(p.x-a.x-t*dx, p.y-a.y-t*dy);
}

inline bool bypassTranslationPathClear(const std::vector<BypassPoint>& path,
                                       const std::vector<BypassPoint>& obstacles,
                                       double yaw, double front, double half_width,
                                       double margin) {
  if (path.empty()) return false;
  const double c=std::cos(yaw), s=std::sin(yaw);
  // Exact segment/expanded-rectangle intersection for a lateral translation
  // with constant heading. A circumscribed circle falsely rejects safe side
  // steps starting only 5 cm from an obstacle near the front edge.
  for (std::size_t i=0;i<path.size();++i) {
    const auto& a=path[i?i-1:0];const auto& b=path[i];
    for (const auto& p:obstacles) {
      const double ax=c*(a.x-p.x)+s*(a.y-p.y);
      const double ay=-s*(a.x-p.x)+c*(a.y-p.y);
      const double bx=c*(b.x-p.x)+s*(b.y-p.y);
      const double by=-s*(b.x-p.x)+c*(b.y-p.y);
      double enter=0.,leave=1.;bool hit=true;
      const double start[2]={ax,ay},delta[2]={bx-ax,by-ay};
      const double limit[2]={front+margin,half_width+margin};
      for (int axis=0;axis<2;++axis) {
        if (std::abs(delta[axis])<1e-12) {
          if (std::abs(start[axis])>limit[axis]) {hit=false;break;}
        } else {
          double lo=(-limit[axis]-start[axis])/delta[axis];
          double hi=(limit[axis]-start[axis])/delta[axis];
          if (lo>hi) std::swap(lo,hi);
          enter=std::max(enter,lo);leave=std::min(leave,hi);
          if (enter>leave) {hit=false;break;}
        }
      }
      if (hit) return false;
    }
  }
  return true;
}

inline bool bypassPathClear(const std::vector<BypassPoint>& path,
                            const std::vector<BypassPoint>& obstacles,
                            double front, double half_width, double margin) {
  if (path.empty()) return false;
  // Circumscribed footprint: safe for every orientation, including corners
  // while rotating. This never shrinks the real rectangular robot footprint.
  const double radius = std::hypot(front, half_width)+margin;
  for (std::size_t i = 0; i < path.size(); ++i) {
    for (const auto& point : obstacles) {
      if (pointSegmentDistance(point, path[i ? i-1 : 0], path[i]) <= radius) return false;
    }
  }
  return true;
}

}  // namespace x3_target_navigation
