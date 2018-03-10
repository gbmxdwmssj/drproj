import rospy
import yaml
from vehicle_state import VehicleState
from vehicle_model import VehicleModel
from virtual_vehicle import VirtualVehicle
from grid_map import GridMap
import matplotlib.image as mpimg

rospy.init_node('test_virtual_vehicle', anonymous=True)

vehicle_state = VehicleState(3.0, 1.0, 90.0)
vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
virtual_vehicle = VirtualVehicle(vehicle_state, vehicle_model, '')

empty = mpimg.imread('/home/kai/catkin_ws/src/drproj/empty.png')
grid_map = GridMap(empty, 0.5)
grid_map.print()
grid_map.show('rviz_global_grid_map')

virtual_vehicle.show('rviz_virtual_vehicle', grid_map)

print('Finished!')