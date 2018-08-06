import rospy
import yaml
import time
from vehicle_state import VehicleState
from vehicle_model import VehicleModel
from virtual_vehicle import VirtualVehicle
from grid_map import GridMap
from moving_obs import MovingObs
import matplotlib.image as mpimg

rospy.init_node('test_virtual_vehicle', anonymous=True)

global_mission = yaml.load(open('/home/kai/catkin_ws/src/drproj/global_mission.yaml'))
vehicle_state = VehicleState(global_mission['start'][0], global_mission['start'][1], global_mission['start'][2])

vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
virtual_vehicle = VirtualVehicle(vehicle_state, vehicle_model, 'vehicle_cmd', 'move_vehicle')

# free = mpimg.imread('/home/kai/catkin_ws/src/drproj/free.png')
# grid_map = GridMap(free, 0.5)
# grid_map.print()
# grid_map.show('rviz_global_grid_map')

moving_obs = MovingObs('/home/kai/catkin_ws/src/drproj/moving_obs.yaml',
                    '/home/kai/catkin_ws/src/drproj/free.yaml',
                    '/home/kai/catkin_ws/src/drproj/free.png')

virtual_vehicle.show('rviz_virtual_vehicle', moving_obs.grid_map)
moving_obs.grid_map.show('rviz_global_grid_map')
rospy.set_param('start_signal', False)
start_signal = rospy.get_param('/start_signal')
r = rospy.Rate(10) # 10hz
while not start_signal and not rospy.core.is_shutdown():
    r.sleep()
    start_signal = rospy.get_param('/start_signal')
    virtual_vehicle.show('rviz_virtual_vehicle', moving_obs.grid_map)
    virtual_vehicle.pub_vehicle_state('virtual_vehicle_state')

while not rospy.core.is_shutdown():
    r.sleep()
    virtual_vehicle.show('rviz_virtual_vehicle', moving_obs.grid_map)
    virtual_vehicle.pub_vehicle_state('virtual_vehicle_state')
    moving_obs.run_once()
    moving_obs.grid_map.show('rviz_global_grid_map')

print('Finished!')