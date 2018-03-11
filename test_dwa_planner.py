import rospy
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

# print(dwa_planner.get_dynamic_window(1.0, -30.0, vehicle_model.config['dt']))

# vehicle_state = VehicleState(1.0, 1.0, 0.0)
# traj = dwa_planner.get_trajectory(vehicle_state, 3.0, 30.0, vehicle_model.config['dt'], 1.0)
# dwa_planner.show_trajectory(traj, 'rviz_predicted_trajectory', grid_map, 'cube')

vehicle_state = VehicleState(1.0, 1.0, 0.0)
traj_cluster = dwa_planner.get_trajectory_cluster(vehicle_state, vehicle_model.config['dt'])
print(len(traj_cluster))

print('Finished!')
