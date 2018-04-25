import numpy as np
from moving_obs import MovingObs
import matplotlib.pyplot as plt
moving_obs = MovingObs('/home/kai/catkin_ws/src/drproj/moving_obs.yaml')




# for key in moving_obs.config:
#     print(moving_obs.config[key])




# print(moving_obs.interpolate_two_pts([0.0,0.0], [1.0,1.0]))


traj_a = np.array(moving_obs.interpolate_multi_pts(moving_obs.config['a']))
plt.plot(traj_a[:,0], traj_a[:,1], 'ro')
plt.show()
# traj_b = np.array(moving_obs.interpolate_multi_pts(moving_obs.config['b']))
# plt.plot(traj_b[:,0], traj_b[:,1])
# plt.show()




# moving_obs.refine()




print('Finished')
