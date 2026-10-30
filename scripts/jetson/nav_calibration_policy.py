"""Reject navigation until the base calibration has physical evidence."""
import json
import math
import sys

def require_calibration(profile):
    if profile.get('status')!='verified' or profile.get('heading_sign_verified') is not True or profile.get('odometry_scale_verified') is not True:
        raise ValueError('Physical base calibration is pending')
    fields=('minimum_stable_linear_mps','measured_position_error_m','xy_goal_tolerance_m','navigation_max_linear_mps')
    values={key:float(profile[key]) for key in fields if profile.get(key) is not None}
    if len(values)!=len(fields) or not all(math.isfinite(v) for v in values.values()):
        raise ValueError('Calibration measurements are missing or invalid')
    if not 0<values['minimum_stable_linear_mps']<=values['navigation_max_linear_mps']<=.1:
        raise ValueError('Measured minimum speed exceeds allowed navigation speed')
    if not 0<=values['measured_position_error_m']<=values['xy_goal_tolerance_m']<=.1 or values['xy_goal_tolerance_m']<=0:
        raise ValueError('Arrival tolerance is inconsistent with measured position error')
    return values

if __name__=='__main__':
    try:
        print(json.dumps(require_calibration(json.load(open(sys.argv[1])))))
    except (ValueError,KeyError,TypeError) as exc:
        print('Navigation disabled: '+str(exc))
        raise SystemExit(2)
