import rospy
import matplotlib.pyplot as plt
from dwa_planner import DWAPlanner
from vehicle_model import VehicleModel
from vehicle_state import VehicleState
from grid_map import GridMap
import matplotlib.image as mpimg

rospy.init_node('test_dwa_planner', anonymous=True)

empty = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')
grid_map = GridMap(empty, 0.5)
grid_map.print()
grid_map.show('rviz_global_grid_map')

vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
dwa_planner = DWAPlanner(vehicle_model, '/home/kai/catkin_ws/src/drproj/dwa_planner_config.yaml')

print('Window: {}'.format(dwa_planner.get_dynamic_window(0.0, 0.0, vehicle_model.config['dt'])))

# vehicle_state = VehicleState(3.0, 0.0, 0.0)
# traj = dwa_planner.get_trajectory(vehicle_state, 1.0, 0.0, vehicle_model.config['dt'], dwa_planner.config['predict_time'])
# print('Size fo trajectory: {}'.format(len(traj)))
# dwa_planner.show_trajectory(traj, 'rviz_predicted_trajectory', grid_map, 'cube')
# print(dwa_planner.orientation_cost(traj, (2.5, 2.0)))
# print(dwa_planner.velocity_cost(traj))
# print('Collision cost: {}'.format(dwa_planner.collision_cost(traj, grid_map)))

# dwa_planner.collision(vehicle_state, grid_map)
# plt.show()

vehicle_state = VehicleState(2.5, 3.0, 0.0, v=0.0, steer=5.0)
traj_cluster = dwa_planner.get_trajectory_cluster(vehicle_state, vehicle_model.config['dt'])
print(len(traj_cluster))
dwa_planner.show_trajectory_cluster(traj_cluster, 'rviz_trajectory_cluster', grid_map)
# print('Orientation costs: {}'.format(dwa_planner.orientation_costs(traj_cluster, (2.0, 3.0))))
# print('Velocity costs: {}'.format(dwa_planner.velocity_costs(traj_cluster)))
print('Collision costs: {}'.format(dwa_planner.collision_costs(traj_cluster, grid_map)))

print('Finished!')
