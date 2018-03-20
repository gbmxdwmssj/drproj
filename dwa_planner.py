import rospy
import yaml
import numpy as np
from math import *
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class DWAPlanner(object):



    def __init__(self, model, config_file):
        # Member
        self.model = model

        f = open(config_file)
        self.config = yaml.load(f)



    def get_dynamic_window(self, v, steer, dt):
        '''
        Arguments
        ---------
            v (float): The current veloticy (m/s).

            steer (float): The current steering angle (degree).
        '''
        # Get related parameters.
        config_v_min = self.model.config['velocity_range'][0]
        config_v_max = self.model.config['velocity_range'][1]
        config_steer_min = self.model.config['steer_range'][0]
        config_steer_max = self.model.config['steer_range'][1]
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



    def get_trajectory(self, state, v, steer, dt, predict_time):
        '''
        Arguments
        ---------
            state (VehicleState): The current state.

        Returns
        -------
            traj (List of VehicleState): The predicted trajectory.
        '''
        traj = []
        traj.append(state)
        t = dt
        while t < predict_time:
            state = self.model.step(state, v, steer, dt)
            traj.append(state)
            t += dt

        return traj



    def show_trajectory(self, traj, topic_name, grid_map, type='arrow'):
        pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10, latch=True)

        msg = MarkerArray()
        for i, state in enumerate(traj):
            arrow = Marker()
            arrow.id = i
            arrow.header.frame_id = '/map'

            if type == 'arrow':
                arrow.type = arrow.ARROW
            else:
                arrow.type = arrow.CUBE

            arrow.action = arrow.ADD
            arrow.pose.position.x = state.x
            arrow.pose.position.y = grid_map.max_y * grid_map.resolution - state.y
            arrow.pose.position.z = 0.0
            tmp_qua = Quaternion(axis=[1, 0, 0], angle=radians(state.yaw - 90.0))
            arrow.pose.orientation.x = tmp_qua[0]
            arrow.pose.orientation.y = tmp_qua[1]
            arrow.pose.orientation.z = tmp_qua[2]
            arrow.pose.orientation.w = tmp_qua[3]

            if type == 'arrow':
                arrow.scale.x = grid_map.resolution
            else:
                arrow.scale.x = 0.05

            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.g = 1.0
            arrow.color.a = 1.0

            msg.markers.append(arrow)

        pub.publish(msg)



    def get_trajectory_cluster(self, state, dt):
        '''
        Returns
        -------
            traj_cluster (List of traj, traj's type is List of VehicleState).
        '''
        traj_cluster = []

        dw = self.get_dynamic_window(state.v, state.steer, dt)
        for iv in np.arange(dw[0], dw[1]+0.0001, self.config['v_resolution']):
            for isteer in np.arange(dw[2], dw[3]+0.0001, self.config['steer_resolution']):
                traj = self.get_trajectory(state, iv, isteer, dt, self.config['predict_time'])
                traj_cluster.append(traj)

        return traj_cluster

    def show_trajectory_cluster(self, traj_cluster, topic_name, grid_map):
        pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10, latch=True)

        id_cnt = 0
        msg = MarkerArray()
        for traj in traj_cluster:
            for state in traj:
                arrow = Marker()
                arrow.id = id_cnt
                arrow.header.frame_id = '/map'
                arrow.type = arrow.CUBE
                arrow.action = arrow.ADD
                arrow.pose.position.x = state.x
                arrow.pose.position.y = grid_map.max_y * grid_map.resolution - state.y
                arrow.pose.position.z = 0.0
                tmp_qua = Quaternion(axis=[1, 0, 0], angle=radians(state.yaw - 90.0))
                arrow.pose.orientation.x = tmp_qua[0]
                arrow.pose.orientation.y = tmp_qua[1]
                arrow.pose.orientation.z = tmp_qua[2]
                arrow.pose.orientation.w = tmp_qua[3]
                arrow.scale.x = 0.05
                arrow.scale.y = 0.05
                arrow.scale.z = 0.05
                arrow.color.g = 1.0
                arrow.color.a = 1.0

                msg.markers.append(arrow)
                id_cnt += 1

        pub.publish(msg)
