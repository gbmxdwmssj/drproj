from dwa_planner import DWAPlanner
from vehicle_model import VehicleModel

vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
dwa_planner = DWAPlanner(vehicle_model)

print(dwa_planner.get_dynamic_window(1.0, -30.0))

print('Finished!')
