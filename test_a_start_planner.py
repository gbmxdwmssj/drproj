from a_star_planner import AStarPlanner
from vehicle_state import VehicleState
import matplotlib.image as mpimg
from grid_map import GridMap

global_planner = AStarPlanner()

start = VehicleState(0.0, 0.0, 0.0)
goal = VehicleState(10.0, 10.0, 0.0)

dr = mpimg.imread('/home/kai/Pictures/dr.png')
global_map = GridMap(dr, 1.0)

path = global_planner.get_path(start, goal, global_map)

print(path.len())

print('Finished!')