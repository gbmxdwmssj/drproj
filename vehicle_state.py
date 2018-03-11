import rospy
from math import *
from visualization_msgs.msg import Marker
from pyquaternion import Quaternion

class VehicleState(object):
    ''' Continuous vehicle state.

    Arguments
    ---------
        x (flaot): From left to right (m).

        y (flaot): From down to up (m).

        yaw (flaot): Down to up is zero. Left negative and right positive, so [-180.0, 180.0) (degree).
    '''



    def __init__(self, x, y, yaw,
            v=0.0, w=0.0,
            a=0.0, ayaw=0.0,
            steer=0.0, vsteer=0.0,
            roll=0.0, vroll=0.0,
            pitch=0.0, vpitch=0.0):
        # Member
        self.x = x
        self.y = y
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch

        self.v = v
        self.steer = steer



    def print(self):
        print('({:.3f}, {:.3f}, {:.3f})'.format(self.x, self.y, self.yaw))



    def show_by_arrow(self, topic_name, grid_map):
        pub = rospy.Publisher(topic_name, Marker, queue_size=10, latch=True)

        arrow = Marker()
        arrow.id = 0
        arrow.header.frame_id = '/map'
        arrow.type = arrow.ARROW
        arrow.action = arrow.ADD
        arrow.pose.position.x = self.x
        arrow.pose.position.y = grid_map.max_y * grid_map.resolution - self.y
        arrow.pose.position.z = 0.0
        tmp_qua = Quaternion(axis=[1, 0, 0], angle=radians(self.yaw - 90.0))
        arrow.pose.orientation.x = tmp_qua[0]
        arrow.pose.orientation.y = tmp_qua[1]
        arrow.pose.orientation.z = tmp_qua[2]
        arrow.pose.orientation.w = tmp_qua[3]
        arrow.scale.x = grid_map.resolution
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color.g = 1.0
        arrow.color.a = 1.0

        pub.publish(arrow)
