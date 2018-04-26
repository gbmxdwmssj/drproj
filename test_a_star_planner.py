import rospy
import yaml
import time
import numpy as np
from a_star_planner import AStarPlanner
from robot_model import RobotModel
import matplotlib.image as mpimg
from grid_map import GridMap

rospy.init_node('test_a_star_planner', anonymous=True)

robot_model = RobotModel(2.0)
global_planner = AStarPlanner(robot_model)

empty = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')
global_map_config = yaml.load(open('/home/kai/catkin_ws/src/drproj/empty.yaml'))
global_map = GridMap(empty, global_map_config['resolution'])
global_map.show('rviz_global_grid_map')
global_map.print()

global_mission = yaml.load(open('/home/kai/catkin_ws/src/drproj/global_mission.yaml'))
start_grid_x = int(global_mission['start'][0] / global_map.resolution + 0.5)
start_grid_y = int(global_mission['start'][1] / global_map.resolution + 0.5)
goal_grid_x = int(global_mission['goal'][0] / global_map.resolution + 0.5)
goal_grid_y = int(global_mission['goal'][1] / global_map.resolution + 0.5)
start = (start_grid_x, start_grid_y)
goal = (goal_grid_x, goal_grid_y)

path = global_planner.get_path(start, goal, global_map)
global_planner.show_path('rviz_global_path', global_map)
print(path)
global_planner.send_path('global_path')

while not rospy.core.is_shutdown():
    time.sleep(1.0)
    global_planner.show_path('rviz_global_path', global_map)

print('Finished!')
