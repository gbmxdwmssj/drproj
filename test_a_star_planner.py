import rospy
from a_star_planner import AStarPlanner
from robot_model import RobotModel
import matplotlib.image as mpimg
from grid_map import GridMap

rospy.init_node('test_a_star_planner', anonymous=True)

robot_model = RobotModel(2.0)
global_planner = AStarPlanner(robot_model)

start = (2, 2)
goal = (14, 14)
empty = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')
global_map = GridMap(empty, 1.0)
global_map.show('rviz_global_grid_map')
global_map.print()

path = global_planner.get_path(start, goal, global_map)
global_planner.show_path('rviz_global_path', global_map)
print(path)

print('Finished!')
