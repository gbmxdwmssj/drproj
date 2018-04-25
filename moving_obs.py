import yaml
import numpy as np
class MovingObs(object):



    def __init__(self, config_file):
        # Member
        f = open(config_file)
        self.config = yaml.load(f)
        self.resolution = 0.1 # m
        # self.



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
        x_resolution = self.resolution * dx / (abs(dx) + abs(dy))
        y_resolution = self.resolution * dy / (abs(dx) + abs(dy))
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
        for key in self.config:
            print(self.config[key])
