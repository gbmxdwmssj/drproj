import rospy
import time
import numpy as np
from moving_obs import MovingObs
import matplotlib.pyplot as plt
moving_obs = MovingObs('/home/kai/catkin_ws/src/drproj/moving_obs.yaml',
                    '/home/kai/catkin_ws/src/drproj/empty.yaml',
                    '/home/kai/catkin_ws/src/drproj/empty.png')
rospy.init_node('test_moving_obs', anonymous=True)



# for key in moving_obs.config:
#     print(moving_obs.config[key])



# print(moving_obs.interpolate_two_pts([0.0,0.0], [1.0,1.0]))



# traj_a = np.array(moving_obs.interpolate_multi_pts(moving_obs.config['a']))
# print(len(traj_a))
# plt.plot(traj_a[:,0], traj_a[:,1], 'ro')
# plt.show()
# traj_b = np.array(moving_obs.interpolate_multi_pts(moving_obs.config['b']))
# plt.plot(traj_b[:,0], traj_b[:,1])
# plt.show()



moving_obs.refine()
print(len(moving_obs.trajs))
for traj in moving_obs.trajs:
    traj = np.array(traj)
    print(len(traj))
    plt.plot(traj[:,0], traj[:,1], 'ro')

plt.show()



# moving_obs.grid_map.show('rviz_global_grid_map')
# time.sleep(3.0)
# moving_obs.run_once()
# moving_obs.grid_map.show('rviz_global_grid_map')



# moving_obs.grid_map.show('rviz_global_grid_map')
# while not rospy.core.is_shutdown():
#     time.sleep(0.5)
#     moving_obs.run_once()
#     moving_obs.grid_map.show('rviz_global_grid_map')



print('Finished')
