"""Translate a measured target position into a map-frame stopping pose."""
import math

def target_parking_pose(robot_pose,target_distance_m,standoff_m):
    x=float(robot_pose['x']);y=float(robot_pose['y']);yaw=float(robot_pose['yaw_rad'])
    distance=float(target_distance_m);standoff=float(standoff_m)
    if not all(math.isfinite(v) for v in (x,y,yaw,distance,standoff)):
        raise ValueError('nonfinite_target_geometry')
    if not 0<standoff<distance or not .1<=distance-standoff<=.8:
        raise ValueError('parking_distance_outside_short_navigation_range')
    c=math.cos(yaw);s=math.sin(yaw)
    return {'frame_id':'map','target':{'x':x+distance*c,'y':y+distance*s},
            'parking':{'x':x+(distance-standoff)*c,'y':y+(distance-standoff)*s,'yaw_rad':yaw},
            'travel_m':distance-standoff,'standoff_m':standoff,
            'target_source':'user_confirmed_distance'}

def choose_parking_candidate(preview,goal,tolerance_m=.02):
    if preview.get('ok') is not True:
        raise ValueError('provider_preview_rejected')
    parking=goal['parking']
    candidates=[o for o in preview.get('options',[])
                if math.hypot(float(o['x'])-parking['x'],float(o['y'])-parking['y'])<=tolerance_m
                and abs(float(o['radius_m'])-goal.get('search_radius_m',goal['travel_m']))<=.001]
    if len(candidates)!=1:
        raise ValueError('no_unique_safe_parking_candidate')
    return candidates[0]
