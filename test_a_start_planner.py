from a_star_planner import AStarPlanner
from robot_model import RobotModel
import matplotlib.image as mpimg
from grid_map import GridMap

robot_model = RobotModel(2.0)
global_planner = AStarPlanner(robot_model)

start = (1, 1)
goal = (10, 10)

empty = mpimg.imread('/home/kai/Pictures/empty.png')
global_map = GridMap(empty, 1.0)
global_map.print()

path = global_planner.get_path(start, goal, global_map)

print(path)

print('Finished!')