import rospy
import time
import matplotlib.pyplot as plt
from dwa_planner import DWAPlanner
from vehicle_model import VehicleModel
from vehicle_state import VehicleState
from grid_map import GridMap
import matplotlib.image as mpimg

goal = (9.0, 9.0)

rospy.init_node('test_dwa_planner', anonymous=True)

empty = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')
grid_map = GridMap(empty, 0.5)
grid_map.print()
grid_map.show('rviz_global_grid_map')

vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
dwa_planner = DWAPlanner(vehicle_model, '/home/kai/catkin_ws/src/drproj/dwa_planner_config.yaml')

# print('Window: {}'.format(dwa_planner.get_dynamic_window(0.0, 0.0, vehicle_model.config['dt'])))



# vehicle_state = VehicleState(3.0, 0.0, 0.0)
# traj = dwa_planner.get_trajectory(vehicle_state, 1.0, 0.0, vehicle_model.config['dt'], dwa_planner.config['predict_time'])
# print('Size fo trajectory: {}'.format(len(traj)))
# dwa_planner.show_trajectory(traj, 'rviz_predicted_trajectory', grid_map, 'cube')
# print(dwa_planner.orientation_cost(traj, goal))
# print(dwa_planner.velocity_cost(traj))
# print('Collision cost: {}'.format(dwa_planner.collision_cost(traj, grid_map)))

# dwa_planner.collision(vehicle_state, grid_map)
# plt.show()



# vehicle_state = VehicleState(2.5, 3.0, 0.0, v=0.0, steer=0.0)
# traj_cluster = dwa_planner.get_trajectory_cluster(vehicle_state, vehicle_model.config['dt'])
# print(len(traj_cluster))
# dwa_planner.show_trajectory_cluster(traj_cluster, 'rviz_trajectory_cluster', grid_map)

# ori_costs = dwa_planner.orientation_costs(traj_cluster, goal)
# vel_costs = dwa_planner.velocity_costs(traj_cluster)
# col_costs = dwa_planner.collision_costs(traj_cluster, grid_map)
# print('Orientation costs: {}\n'.format(ori_costs))
# print('Velocity costs: {}\n'.format(vel_costs))
# print('Collision costs: {}\n'.format(col_costs))

# normed_ori_costs = dwa_planner.normalize_costs(ori_costs)
# normed_vel_costs = dwa_planner.normalize_costs(vel_costs)
# normed_col_costs = dwa_planner.normalize_costs(col_costs)
# print('Normalized orientation costs: {}\n'.format(normed_ori_costs))
# print('Normalized velocity costs: {}\n'.format(normed_vel_costs))
# print('Normalized collision costs: {}\n'.format(normed_col_costs))

# best_traj = dwa_planner.get_best_trajectory(traj_cluster, goal, grid_map)
# dwa_planner.show_trajectory(best_traj, 'rviz_predicted_trajectory', grid_map, 'cube')
# v = best_traj[1].v
# steer = best_traj[1].steer

while not rospy.core.is_shutdown():
    # time.sleep(0.1)
    dwa_planner.run_once(goal, grid_map)
    # dwa_planner.send_cmd('vehicle_cmd', v, steer)

print('Finished!')
