import yaml
from moving_obs import MovingObs

moving_obs = MovingObs('/home/kai/catkin_ws/src/drproj/moving_obs.yaml')

# for key in moving_obs.config:
#     print(moving_obs.config[key])

print(moving_obs.interpolate([0.0,0.0], [1.0,1.0]))

print('Finished')
