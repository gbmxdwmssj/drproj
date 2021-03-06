import rospy
from math import *
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray
from drproj.srv import *

class VirtualVehicle(object):



    def __init__(self, state, model, cmd_topic_name, cmd_srv_name, is_ros=True):
        # Member
        self.state = state
        self.model = model
        if is_ros:
            self.cmd_sub = rospy.Subscriber(cmd_topic_name, AckermannDriveStamped, self.cmd_cb)
            self.cmd_srv = rospy.Service(cmd_srv_name, MoveVehicle, self.move_vehicle)
            self.reset_srv = rospy.Service('reset_vehicle', ResetVehicle, self.reset_vehicle)



    def cmd_cb(self, data):
        v = data.drive.speed
        steer = data.drive.steering_angle
        self.step(v, steer, self.model.config['dt'])



    def move_vehicle(self, req):
        for _ in range(self.model.config['steps_per_move']):
            self.step(req.v, req.steer, self.model.config['dt'])
        return MoveVehicleResponse(x=self.state.x, y=self.state.y, yaw=self.state.yaw,
                                v=req.v, steer=req.steer)



    def reset_vehicle_manually(self, state):
        self.state.x = state[0]
        self.state.y = state[1]
        self.state.yaw = state[2]
        self.state.v = state[3]
        self.state.steer = state[4]
        # print('-------------------- reset in virtual vehicle manually --------------------')



    def reset_vehicle(self, req):
        self.state.x = req.state[0]
        self.state.y = req.state[1]
        self.state.yaw = req.state[2]
        self.state.v = req.state[3]
        self.state.steer = req.state[4]
        print('-------------------- reset in virtual vehicle --------------------')
        return True



    def step(self, v, steer, dt):
        ''' Update the virtual vehicle's state according to the input control.

        Arguments
        ---------
            v (float): m/s.

            steer (float): degree.

            dt (float): s.
        '''
        self.state = self.model.step(self.state, v, steer, dt)
        return self.state



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



    def pub_vehicle_state(self, topic_name):
        pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
        vehicle_state_msg = Float64MultiArray()
        vehicle_state_msg.data.append(self.state.x)
        vehicle_state_msg.data.append(self.state.y)
        vehicle_state_msg.data.append(self.state.yaw)
        vehicle_state_msg.data.append(self.state.v)
        vehicle_state_msg.data.append(self.state.steer)
        pub.publish(vehicle_state_msg)
