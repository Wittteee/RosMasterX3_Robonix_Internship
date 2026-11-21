#include "x3_target_navigation/bypass_geometry.hpp"
#include <cassert>
#include <limits>

using namespace x3_target_navigation;

int main() {
  const auto decide = [](std::vector<BypassPoint> points) {
    return classifyBypass(points, 1.05, .117, .100, .05, .35, .45);
  };
  // Clear centreline, but the right body edge would hit: go LEFT.
  auto right = decide({{.4,-.13},{.4,-.10},{.4,-.07}});
  assert(right.blocked && right.side == BypassSide::LEFT && right.shift_y > 0.);
  auto left = decide({{.4,.07},{.4,.10},{.4,.13}});
  assert(left.blocked && left.side == BypassSide::RIGHT && left.shift_y < 0.);
  assert(!decide({{.4,.20}}).blocked);
  assert(!decide({{.8,-.09}}).blocked);  // Not within the reaction lookahead yet.
  auto full = decide({{.4,-.12},{.4,0.},{.4,.12}});
  assert(full.blocked && full.full_width && full.side == BypassSide::NONE);
  assert(decide({{.4,-.03},{.4,.03}}).side == BypassSide::NONE);
  assert(!decide({{-.09,.08}}).blocked);
  assert(!decide({{std::numeric_limits<double>::quiet_NaN(),0.}}).blocked);

  const auto ten_cm = [](double gap) {
    return classifyBypass({{.117+gap,.07},{.117+gap,.10},{.117+gap,.13}},
                          1.05,.117,.1,.05,.10,.45);
  };
  assert(!ten_cm(.101).blocked);
  assert(ten_cm(.100).blocked && ten_cm(.100).side == BypassSide::RIGHT);
  assert(ten_cm(.099).blocked && ten_cm(.099).side == BypassSide::RIGHT);
  assert(classifyBypass({{.216,-.13},{.216,-.07}},1.05,.117,.1,.05,.1,.45).side == BypassSide::LEFT);
  const auto five_cm = [](double gap) {
    return classifyBypass({{.117+gap,.07},{.117+gap,.10},{.117+gap,.13}},
                          1.05,.117,.1,.05,.05,.45);
  };
  assert(!five_cm(.051).blocked);
  assert(five_cm(.050).blocked && five_cm(.050).side == BypassSide::RIGHT);
  assert(five_cm(.049).blocked && five_cm(.049).side == BypassSide::RIGHT);
  // The leading tip alone is right of centre; the nearby face is on the left.
  assert(classifyBypass({{.167,-.01},{.175,.07},{.18,.14}},1.05,.117,.1,.05,.05,.45).side == BypassSide::RIGHT);
  // Gathering the rest of a face must never trigger before the 5 cm boundary.
  assert(!classifyBypass({{.168,-.01},{.175,.07},{.18,.14}},1.05,.117,.1,.05,.05,.45).blocked);
  // Gap is measured from the actual body front even when route/body axes differ.
  const double heading=.2,bx=.166,by=.07;
  BypassPoint rotated{std::cos(heading)*bx-std::sin(heading)*by,
                      std::sin(heading)*bx+std::cos(heading)*by};
  assert(classifyBypass({rotated},1.05,.117,.1,.05,.05,.45,heading).blocked);

  // Close bottle: keep the same side but enlarge the shift to preserve the
  // parking standoff and a useful final approach. Never exceed the lane bound.
  assert(bypassShiftForTarget(right,{1.05,0.},.297,.45)==right.shift_y);
  const double close_left = bypassShiftForTarget(right,{right.pass_x,0.},.297,.45);
  const double close_right = bypassShiftForTarget(left,{left.pass_x,0.},.297,.45);
  assert(close_left >= .377 && close_right <= -.377);
  assert(std::isnan(bypassShiftForTarget(right,{right.pass_x,.2},.297,.45)));
  assert(std::isnan(bypassShiftForTarget(full,{1.,0.},.297,.45)));
  assert(isChassisReturn(-.1185,.01,.117,.1));
  assert(!isChassisReturn(-.122,.01,.117,.1));
  assert(!isChassisReturn(-.1185,.05,.117,.1));
  assert(!isChassisReturn(.1185,.01,.117,.1));
  assert(!isChassisReturn(0.,.101,.117,.1));

  assert(insideSweptFootprint({.10,.14}, 0.,.04,0., .117,.1,.03));
  assert(insideSweptFootprint({.10,-.14}, 0.,-.04,0., .117,.1,.03));
  assert(!insideSweptFootprint({.10,-.20}, 0.,.03,0., .117,.1,.03));
  assert(insideSweptFootprint({.16,0.}, .04,0.,0., .117,.1,.03));
  assert(insideSweptFootprint({-.16,0.}, -.04,0.,0., .117,.1,.03));
  assert(insideSweptFootprint({0.,0.}, std::numeric_limits<double>::quiet_NaN(),0.,0., .117,.1,.03));

  assert(bypassLegAllowed(BypassSide::LEFT,0,0.,.20,0.,.20));
  assert(!bypassLegAllowed(BypassSide::LEFT,0,.1,.20,0.,.20));
  assert(!bypassLegAllowed(BypassSide::LEFT,0,0.,-.10,0.,.20));
  assert(bypassLegAllowed(BypassSide::LEFT,0,-.05,-.05,0.,.20));
  assert(!bypassLegAllowed(BypassSide::LEFT,0,.15,-.05,0.,.20));
  assert(bypassLegAllowed(BypassSide::RIGHT,0,0.,-.20,0.,-.20));
  assert(!bypassLegAllowed(BypassSide::RIGHT,0,0.,.10,0.,-.20));
  assert(!bypassLegAllowed(BypassSide::LEFT,1,.5,.1,0.,.20));
  assert(bypassLegAllowed(BypassSide::LEFT,1,.5,.20,0.,.20));
  assert(bypassLegAllowed(BypassSide::LEFT,1,.5,.15,0.,.20));
  assert(bypassLegAllowed(BypassSide::LEFT,2,.7,.05,0.,.20));

  std::vector<BypassPoint> path{{0.,0.},{0.,.2},{.7,.2}};
  assert(bypassPathClear(path,{{.4,-.09}},.117,.1,.03));
  assert(!bypassPathClear(path,{{0.,.25}},.117,.1,.03));  // Left side occupied.
  assert(!bypassPathClear({{0.,0.},{.7,0.}},{{.4,-.09}},.117,.1,.03));
  assert(!bypassPathClear({}, {},.117,.1,.03));
  // Rectangle corner would hit even when a narrow centreline-only test passes.
  assert(!bypassPathClear({{0.,0.},{.4,0.}},{{.4,.15}},.117,.1,.03));
  std::vector<BypassPoint> side_step{{0.,0.},{0.,-.2}};
  assert(!bypassPathClear(side_step,{{.167,.05}},.117,.1,.03));
  assert(bypassTranslationPathClear(side_step,{{.167,.05}},0.,.117,.1,.03));
  assert(!bypassTranslationPathClear(side_step,{{.14,.05}},0.,.117,.1,.03));
  assert(!bypassTranslationPathClear(side_step,{{0.,-.22}},0.,.117,.1,.03));
  assert(!bypassTranslationPathClear(side_step,{{-.14,-.1}},0.,.117,.1,.03));
  assert(bypassTranslationPathClear({{0.,0.},{.2,0.}},{{-.05,.167}},std::acos(-1.)/2,.117,.1,.03));
  assert(!bypassTranslationPathClear({}, {},0.,.117,.1,.03));
}
