"""In-memory safe_move contract substitute, never a live fallback."""
import math
import time

class SimulationProvider:
    """No transport, no ROS imports, no physical actuation."""
    def health(self):
        return {"ok": True, "simulation": True}

    def preview(self):
        return {"ok": True, "session_id": "simulation", "expires_epoch": time.time() + 60,
                "current_pose": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
                "options": [{"option_id": n, "radius_m": 0.2, "relative_angle_deg": angle,
                             "x": .2*math.cos(math.radians(angle)), "y": .2*math.sin(math.radians(angle)), "yaw_deg": 0.0}
                            for n, angle in enumerate((-20, 0, 20), 1)]}

    def prepare(self, session_id, option_id):
        return {"ok": True, "token": "simulation-token", "confirmation_text": "simulation",
                "expires_at_epoch": time.time() + 30}

    def execute(self, token, confirmation_text):
        return {"ok": True, "arrived": True, "move_base_status_name": "SUCCEEDED",
                "simulation": True}

    def cancel(self):
        return {"ok": True, "simulation": True}

    def close(self):
        pass
