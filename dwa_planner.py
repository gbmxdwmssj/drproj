import rospy
import yaml
import numpy as np
from math import *
from vehicle_state import VehicleState
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int64
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from moving_obs import MovingObs

class DWAPlanner(object):



    def __init__(self, model, config_file):
        # Member
        self.model = model

        f = open(config_file)
        self.config = yaml.load(f)
        f = open('/home/kai/catkin_ws/src/drproj/empty.yaml')
        self.global_map_config = yaml.load(f)
        self.moving_obs = MovingObs('/home/kai/catkin_ws/src/drproj/moving_obs.yaml',
                            '/home/kai/catkin_ws/src/drproj/empty.yaml',
                            '/home/kai/catkin_ws/src/drproj/empty.png', is_clear=True, is_pub_id=False)

        self.small_num = 0.0001

        self.vehicle_state = VehicleState(0.0, 0.0, 0.0)
        self.global_path_grid = Path()
        self.global_path_meter = Path()
        self.mov_id = 0

        self.vehicle_state_sub = rospy.Subscriber('virtual_vehicle_state', Float64MultiArray, self.vehicle_state_cb)
        self.global_path_sub = rospy.Subscriber('global_path', Path, self.global_path_cb)
        self.mov_id_sub = rospy.Subscriber('cur_moving_obs_id', Int64, self.mov_id_cb)



    def vehicle_state_cb(self, data):
        self.vehicle_state.x = data.data[0]
        self.vehicle_state.y = data.data[1]
        self.vehicle_state.yaw = data.data[2]
        self.vehicle_state.v = data.data[3]
        self.vehicle_state.steer = data.data[4]



    def global_path_cb(self, data):
        self.global_path_grid = data
        for ps_grid in data.poses:
            ps_meter = PoseStamped()
            ps_meter.pose.position.x = ps_grid.pose.position.x * self.global_map_config['resolution']
            ps_meter.pose.position.y = ps_grid.pose.position.y * self.global_map_config['resolution']
            self.global_path_meter.poses.append(ps_meter)



    def mov_id_cb(self, data):
        self.mov_id = data.data



    def get_prospect(self):
        # Check
        if len(self.global_path_meter.poses) == 0:
            return (self.vehicle_state.x, self.vehicle_state.y)

        # Match
        dis_list = []
        for ps in self.global_path_meter.poses:
            dx = ps.pose.position.x - self.vehicle_state.x
            dy = ps.pose.position.y - self.vehicle_state.y
            dis = sqrt(dx**2 + dy**2)
            dis_list.append(dis)

        match_id = np.argmin(dis_list)

        # Prospect
        delta_id = int(self.config['prospect_dis'] / self.global_map_config['resolution'] + 0.5)
        prospect_id = match_id + delta_id
        prospect_id = min(prospect_id, len(self.global_path_meter.poses)-1)
        prospect_x = self.global_path_meter.poses[prospect_id].pose.position.x
        prospect_y = self.global_path_meter.poses[prospect_id].pose.position.y
        return (prospect_x, prospect_y)



    def normalize_costs(self, costs):
        sum_cost = sum(costs)
        if sum_cost == 0:
            return [0] * len(costs)

        normed_costs = []
        for cost in costs:
            normed_costs.append(1.0 * cost / sum_cost)

        return normed_costs



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
        while t < predict_time + self.small_num:
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
            arrow.pose.position.z = -0.01
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
            arrow.color.r = 1.0
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
        for iv in np.arange(dw[0], dw[1]+self.small_num, self.config['v_resolution']):
            for isteer in np.arange(dw[2], dw[3]+self.small_num, self.config['steer_resolution']):
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



    def orientation_cost(self, traj, goal):
        '''
        Augrments
        ---------
            traj (List of VehicleState): The trajectory that needs to be evaluated.

            goal (tuple): The goal point.

        Returns
        -------
            _ (float): The orientation cost of the input trajectory.
        '''
        x = traj[-1].x
        y = traj[-1].y
        yaw = traj[-1].yaw

        dx = goal[0] - x
        dy = goal[1] - y
        target_yaw = degrees(atan2(dx, dy))

        delta_yaw = target_yaw - yaw
        while delta_yaw <= -180.0:
            delta_yaw += 360.0
        while delta_yaw > 180.0:
            delta_yaw -= 360.0

        return fabs(delta_yaw)



    def orientation_costs(self, traj_cluster, goal):
        costs = []
        for traj in traj_cluster:
            costs.append(self.orientation_cost(traj, goal))

        return costs



    def distance_cost(self, traj, goal):
        dx = goal[0] - traj[-1].x
        dy = goal[1] - traj[-1].y
        dis_to_goal = sqrt(dx**2 + dy**2)
        return dis_to_goal



    def distance_costs(self, traj_cluster, goal):
        costs = []
        for traj in traj_cluster:
            costs.append(self.distance_cost(traj, goal))

        return costs



    def velocity_cost(self, traj):
        abs_v = fabs(traj[-1].v)
        abs_v = max(abs_v, self.small_num)
        return 1.0 / abs_v



    def velocity_costs(self, traj_cluster):
        costs = []
        for traj in traj_cluster:
            costs.append(self.velocity_cost(traj))

        return costs



    def collision_cost(self, traj, grid_map):
        for i, state in enumerate(traj):
            if self.collision(state, grid_map):
                # For simplity, use index as the time stamp.
                t_col = (i + 1) * self.model.config['dt']
                return 1.0 / t_col

        # No collision, so the time of collision is infinite.
        t_col = float('inf')
        return 1.0 / t_col



    def collision_costs(self, traj_cluster, grid_map):
        costs = []
        for traj in traj_cluster:
            costs.append(self.collision_cost(traj, grid_map))

        return costs



    def half_collision(self, grid, grid_map, radius):
        radius_in_grid = int(ceil(radius / grid_map.resolution))
        for dx in range(-radius_in_grid, radius_in_grid+1):
            for dy in range(-radius_in_grid, radius_in_grid+1):
                ddis_in_grid = dx**2 + dy**2
                if ddis_in_grid > radius_in_grid**2:
                    continue

                # Here is grid within the circle that needs to be checked for collision.
                x = grid[0] + dx
                y = grid[1] + dy
                # plt.plot(x, y, 'ro')
                if grid_map.grid_type(x, y) == 'occupied':
                    return True

        return False



    def collision(self, state, grid_map):
        x = 0.5 * self.model.config['axis_length']
        y = 0.25 * self.model.config['length']
        check_radius = sqrt(x**2 + y**2)

        # Rear point.
        edge = 0.5 * self.model.config['wheelbase'] - 0.25 * self.model.config['length']
        rear_x = state.x + edge * sin(radians(state.yaw))
        rear_y = state.y + edge * cos(radians(state.yaw))
        grid_x = int(rear_x / grid_map.resolution + 0.5)
        grid_y = int(rear_y / grid_map.resolution + 0.5)
        # grid_x = np.clip(grid_x, 0, grid_map.max_x-1)
        # grid_y = np.clip(grid_y, 0, grid_map.max_y-1)
        grid = (grid_x, grid_y)
        if self.half_collision(grid, grid_map, check_radius):
            return True

        # Front point.
        edge = 0.5 * self.model.config['wheelbase'] + 0.25 * self.model.config['length']
        front_x = state.x + edge * sin(radians(state.yaw))
        front_y = state.y + edge * cos(radians(state.yaw))
        grid_x = int(front_x / grid_map.resolution + 0.5)
        grid_y = int(front_y / grid_map.resolution + 0.5)
        # grid_x = np.clip(grid_x, 0, grid_map.max_x-1)
        # grid_y = np.clip(grid_y, 0, grid_map.max_y-1)
        grid = (grid_x, grid_y)
        if self.half_collision(grid, grid_map, check_radius):
            return True

        return False



    def moving_collision_cost(self, traj):
        self.moving_obs.cur_id = self.mov_id
        for i, state in enumerate(traj):
            self.moving_obs.run_once()
            if self.collision(state, self.moving_obs.grid_map):
                # For simplity, use index as the time stamp.
                t_col = (i + 1) * self.model.config['dt']
                return 1.0 / t_col

        # No collision, so the time of collision is infinite.
        t_col = float('inf')
        return 1.0 / t_col



    def get_best_trajectory(self, traj_cluster, goal, grid_map):
        normed_ori_costs = self.normalize_costs(self.orientation_costs(traj_cluster, goal))
        normed_dis_costs = self.normalize_costs(self.distance_costs(traj_cluster, goal))
        normed_vel_costs = self.normalize_costs(self.velocity_costs(traj_cluster))
        normed_col_costs = self.normalize_costs(self.collision_costs(traj_cluster, grid_map))

        weighted_ori_costs = np.multiply(self.config['w_ori'], normed_ori_costs)
        weighted_dis_costs = np.multiply(self.config['w_dis'], normed_dis_costs)
        weighted_vel_costs = np.multiply(self.config['w_vel'], normed_vel_costs)
        weighted_col_costs = np.multiply(self.config['w_col'], normed_col_costs)
        normed_total_costs = weighted_ori_costs + weighted_dis_costs + weighted_vel_costs + weighted_col_costs
        best_traj = traj_cluster[np.argmin(normed_total_costs)]
        return best_traj



    def send_cmd(self, topic_name, v, steer):
        cmd_pub = rospy.Publisher(topic_name, AckermannDriveStamped, queue_size=10)
        cmd_msg = AckermannDriveStamped()
        cmd_msg.drive.speed = v
        cmd_msg.drive.steering_angle = steer
        cmd_pub.publish(cmd_msg)



    def run_once(self, grid_map):
        goal = self.get_prospect()
        traj_cluster = self.get_trajectory_cluster(self.vehicle_state, self.model.config['dt'])
        best_traj = self.get_best_trajectory(traj_cluster, goal, grid_map)
        self.show_trajectory(best_traj, 'rviz_predicted_trajectory', grid_map, 'cube')
        v = best_traj[1].v
        steer = best_traj[1].steer
        self.send_cmd('vehicle_cmd', v, steer)
