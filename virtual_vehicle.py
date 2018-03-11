import rospy
from math import *
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

class VirtualVehicle(object):



    def __init__(self, state, model, cmd_topic_name):
        # Member
        self.state = state
        self.model = model
        self.cmd_sub = rospy.Subscriber(cmd_topic_name, AckermannDriveStamped, self.cmd_cb)



    def cmd_cb(self, data):
        v = data.drive.speed
        steer = data.drive.steering_angle
        self.step(v, steer, self.model.config['dt'])



    def step(self, v, steer, dt):
        ''' Update the virtual vehicle's state according to the input control.

        Arguments
        ---------
            v (float): m/s.

            steer (float): degree.

            dt (float): s.
        '''
        self.state = self.model.step(self.state, v, steer, dt)



    def show(self, topic_name, grid_map):
        pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10, latch=True)

        # Build marker for center point of rear axis.
        center_rear = Marker()
        center_rear.id = 0
        center_rear.header.frame_id = '/map'
        center_rear.type = center_rear.CUBE
        center_rear.action = center_rear.ADD
        center_rear.pose.position.x = self.state.x
        center_rear.pose.position.y = grid_map.max_y * grid_map.resolution - self.state.y
        center_rear.pose.position.z = -0.05
        center_rear.scale.x = 0.1
        center_rear.scale.y = 0.1
        center_rear.scale.z = 0.1
        center_rear.color.r = 1.0
        center_rear.color.a = 1.0
        
        # Build marker for center point of vehicle.
        center = Marker()
        center.id = 1
        center.header.frame_id = '/map'
        center.type = center.CUBE
        center.action = center.ADD
        center.pose.position.x = self.state.x + 0.5 * self.model.config['wheelbase'] * sin(radians(self.state.yaw))
        center.pose.position.y = grid_map.max_y * grid_map.resolution - (self.state.y + 0.5 * self.model.config['wheelbase'] * cos(radians(self.state.yaw)))
        center.pose.position.z = -0.05
        center.scale.x = 0.1
        center.scale.y = 0.1
        center.scale.z = 0.1
        center.color.g = 1.0
        center.color.a = 1.0

        # Build marker for vehicle.
        vehicle = Marker()
        vehicle.id = 2
        vehicle.header.frame_id = '/map'
        vehicle.type = vehicle.CUBE
        vehicle.action = vehicle.ADD
        vehicle.pose.position.x = self.state.x + 0.5 * self.model.config['wheelbase'] * sin(radians(self.state.yaw))
        vehicle.pose.position.y = grid_map.max_y * grid_map.resolution - (self.state.y + 0.5 * self.model.config['wheelbase'] * cos(radians(self.state.yaw)))
        vehicle.pose.position.z = 0.005
        tmp_qua = Quaternion(axis=[1, 0, 0], angle=radians(self.state.yaw))
        vehicle.pose.orientation.x = tmp_qua[0]
        vehicle.pose.orientation.y = tmp_qua[1]
        vehicle.pose.orientation.z = tmp_qua[2]
        vehicle.pose.orientation.w = tmp_qua[3]
        vehicle.scale.x = self.model.config['width']
        vehicle.scale.y = self.model.config['length']
        vehicle.scale.z = 0.01
        vehicle.color.b = 1.0
        vehicle.color.a = 1.0

        # Build message for state.
        msg = MarkerArray()
        msg.markers.append(center_rear)
        msg.markers.append(center)
        msg.markers.append(vehicle)

        pub.publish(msg)
