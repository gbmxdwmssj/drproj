import matplotlib.image as mpimg
from grid_map import GridMap
from robot_model import RobotModel

dr = mpimg.imread('/home/kai/Pictures/dr.png')

global_grid_map = GridMap(dr, 1.0)
global_grid_map.print()

robot_model = RobotModel(2.0)
controls = robot_model.get_controls((639,200), global_grid_map)
print(controls)
