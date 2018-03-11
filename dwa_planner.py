import numpy as np

class DWAPlanner(object):



    def __init__(self, model):
        # Member
        self.model = model



    def get_dynamic_window(self, v, steer):
        # Get related parameters.
        config_v_min = self.model.config['velocity_range'][0]
        config_v_max = self.model.config['velocity_range'][1]
        config_steer_min = self.model.config['steer_range'][0]
        config_steer_max = self.model.config['steer_range'][1]
        dt = self.model.config['dt']
        max_deceleration = self.model.config['acceleration_range'][0]
        max_acceleartion = self.model.config['acceleration_range'][1]
        max_neg_v_steer = self.model.config['v_steer_range'][0]
        max_pos_v_steer = self.model.config['v_steer_range'][1]

        # Process inputs.
        v = np.clip(v, config_v_min, config_v_max)
        steer = np.clip(steer, config_steer_min, config_steer_max)

        # Dynamic window from actual motion state.
        actual_v_min = v + max_deceleration * dt
        actual_v_max = v + max_acceleartion * dt
        actual_steer_min = steer + max_neg_v_steer * dt
        actual_steer_max = steer + max_pos_v_steer * dt

        # Final dynamic window.
        final_v_min = max(config_v_min, actual_v_min)
        final_v_max = min(config_v_max, actual_v_max)
        final_steer_min = max(config_steer_min, actual_steer_min)
        final_steer_max = min(config_steer_max, actual_steer_max)

        return (final_v_min, final_v_max, final_steer_min, final_steer_max)
