from math import *
import yaml
from vehicle_state import VehicleState

class VehicleModel(object):
    '''
    Arguments
    ---------
        config_file (str): The directory of vechile configuration file.
    '''



    def __init__(self, config_file):
        f = open(config_file)
        self.config = yaml.load(f)



    def step(self, s0, v, steer, dt):
        '''
        Arguments
        ---------
            s0 (VehicleState): The current vehicle state (m, m, degree).

            v (float): m/s.

            steer (float): degree.

            dt (float): s.

        Returns
        -------
            s1 (VehicleState): The next vehicle state (m, m, degree).
        '''
        s1x = s0.x + v * sin(radians(s0.yaw)) * dt
        s1y = s0.y + v * cos(radians(s0.yaw)) * dt
        s1yaw = s0.yaw + degrees(v / self.config['wheelbase'] * tan(radians(steer)) * dt)

        while s1yaw >= 180.0:
            s1yaw -= 360.0
        while s1yaw < -180.0:
            s1yaw += 360.0

        s1 = VehicleState(s1x, s1y, s1yaw, v=v, steer=steer)
        return s1
