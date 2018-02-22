import rospy
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from grid_map import GridMap

rospy.init_node('test_grid_map', anonymous=True)
dr = mpimg.imread('/home/kai/Pictures/dr.png')

global_grid_map = GridMap(dr, 1.0)
global_grid_map.show('rviz_global_grid_map')

print(global_grid_map.occupancy(1, 1))

global_grid_map.print()