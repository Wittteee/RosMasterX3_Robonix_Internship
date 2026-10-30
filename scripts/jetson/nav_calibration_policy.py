"""Reject navigation until the base calibration has physical evidence."""
import json
import math
import sys

def require_speed_limits(profile, linear_override=None, angular_override=None):
    """Shared metric limits; usable by the vendor's Python 2 interpreter.

    ROS overrides may lower the profile limits, but never silently raise them.
    This validates software limits only, not physical speed calibration.
    """
    def positive(value, ceiling):
        if isinstance(value, bool):
            raise ValueError('Speed limits must be numeric, not boolean')
        value = float(value)
        if math.isnan(value) or math.isinf(value) or not 0 < value <= ceiling:
            raise ValueError('Speed limit exceeds allowed range')
        return value
    if profile.get('velocity_mapping', 'metric_identity_with_limits') != 'metric_identity_with_limits':
        raise ValueError('Unsupported velocity mapping')
    linear = positive(profile.get('max_linear_mps', .1), .1)
    angular = positive(profile.get('max_angular_rps', .2), .2)
    if linear_override is not None:
        linear = positive(linear_override, linear)
    if angular_override is not None:
        angular = positive(angular_override, angular)
    return {'max_linear_mps': linear, 'max_angular_rps': angular}

def require_calibration(profile):
    limits = require_speed_limits(profile)
    if profile.get('status')!='verified' or profile.get('heading_sign_verified') is not True or profile.get('odometry_scale_verified') is not True:
        raise ValueError('Physical base calibration is pending')
    fields=('minimum_stable_linear_mps','measured_position_error_m','xy_goal_tolerance_m','navigation_max_linear_mps')
    values={key:float(profile[key]) for key in fields if profile.get(key) is not None}
    if len(values)!=len(fields) or not all(math.isfinite(v) for v in values.values()):
        raise ValueError('Calibration measurements are missing or invalid')
    if not 0<values['minimum_stable_linear_mps']<=values['navigation_max_linear_mps']<=limits['max_linear_mps']:
        raise ValueError('Measured minimum speed exceeds allowed navigation speed')
    if not 0<=values['measured_position_error_m']<=values['xy_goal_tolerance_m']<=.1 or values['xy_goal_tolerance_m']<=0:
        raise ValueError('Arrival tolerance is inconsistent with measured position error')
    values.update(limits)
    return values

if __name__=='__main__':
    try:
        print(json.dumps(require_calibration(json.load(open(sys.argv[1])))))
    except (ValueError,KeyError,TypeError) as exc:
        print('Navigation disabled: '+str(exc))
        raise SystemExit(2)
