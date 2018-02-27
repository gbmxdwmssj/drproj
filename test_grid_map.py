import rospy
import matplotlib.image as mpimg
from grid_map import GridMap

rospy.init_node('test_grid_map', anonymous=True)
dr = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')

global_grid_map = GridMap(dr, 1.0)
global_grid_map.show('rviz_global_grid_map')

print(global_grid_map.occupancy(1, 1))

global_grid_map.print()
