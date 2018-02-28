from vehicle_model import VehicleModel

vehicle_model = VehicleModel('/home/kai/catkin_ws/src/drproj/vehicle_config.yaml')
print(vehicle_model.config)
print(vehicle_model.config['velocity_range'][1])
print(vehicle_model.config['width'])
print(vehicle_model.config['wheelbase'])
