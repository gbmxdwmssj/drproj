import rospy
import yaml
from math import *
import numpy as np
import matplotlib.image as mpimg
from grid_map import GridMap
from std_msgs.msg import Int64

class MovingObs(object):



    def __init__(self, config_file, map_config_file, map_file, is_clear=False, is_pub_id=True, resolution=0.5):
        # Member
        f = open(config_file)
        self.config = yaml.load(f)

        f = open(map_config_file)
        self.map_config = yaml.load(f)

        map_array = mpimg.imread(map_file)
        self.grid_map = GridMap(map_array, self.map_config['resolution'])
        self.grid_map.print()

        self.resolution = resolution # m
        self.trajs = []
        self.last_id = None
        self.cur_id = 0
        self.is_pub_id = is_pub_id

        # Refine
        self.refine()

        # Clear grid map.
        if is_clear:
            self.clear_map()

        # Spawn obstacles into the map.
        for traj in self.trajs:
            x = int(traj[self.cur_id][0] / self.grid_map.resolution + 0.5)
            y = int(traj[self.cur_id][1] / self.grid_map.resolution + 0.5)
            self.grid_map.set_grid(x, y, 1.0)

        self.last_id = self.cur_id
        self.cur_id += 1



    def interpolate_two_pts(self, start, end):
        '''
        Arguments
        ---------
            start (A 'point' represented by list, meter)

            end (A 'point' represented by list, meter)

        Returns
        -------
            _ (A list of 'point', including 'start' but excluding 'end')
        '''
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if abs(dx) + abs(dy) == 0:
            return []

        x_resolution = self.resolution * dx / sqrt(dx**2 + dy**2)
        y_resolution = self.resolution * dy / sqrt(dx**2 + dy**2)
        max_dis = np.linalg.norm(np.subtract(end, start))
        pts = []

        pt = start.copy()
        while np.linalg.norm(np.subtract(pt, start)) < max_dis:
            pts.append(pt.copy())
            pt[0] += x_resolution
            pt[1] += y_resolution

        return pts



    def interpolate_multi_pts(self, multi_pts):
        pts = []
        for i in range(len(multi_pts)-1):
            start = multi_pts[i]
            end = multi_pts[i+1]
            pts += self.interpolate_two_pts(start, end)

        return pts



    def refine(self):
        self.trajs = []
        for key in self.config:
            self.trajs.append(self.interpolate_multi_pts(self.config[key]))



    def run_once(self):
        if self.is_pub_id:
            pub = rospy.Publisher('cur_moving_obs_id', Int64, queue_size=10, latch=True)
            pub.publish(Int64(data=self.cur_id))

        for traj in self.trajs:
            i = min(self.last_id, len(traj)-1)
            x = int(traj[i][0] / self.grid_map.resolution + 0.5)
            y = int(traj[i][1] / self.grid_map.resolution + 0.5)
            self.grid_map.set_grid(x, y, 0.0)

            i = min(self.cur_id, len(traj)-1)
            x = int(traj[i][0] / self.grid_map.resolution + 0.5)
            y = int(traj[i][1] / self.grid_map.resolution + 0.5)
            self.grid_map.set_grid(x, y, 1.0)

        self.last_id = self.cur_id
        self.cur_id += 1



    def clear_map(self):
        for x in range(self.grid_map.max_x):
            for y in range(self.grid_map.max_y):
                self.grid_map.set_grid(x, y, 0.0)
