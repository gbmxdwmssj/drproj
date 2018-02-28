import rospy
from math import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class VirtualVehicle(object):



    def __init__(self, state, model, cmd_topic_name):
        # Member
        self.state = state
        self.model = model
        # sub = rospy.Subscriber(cmd_topic_name, )



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
        center_rear.pose.position.z = -0.5
        center_rear.scale.x = 0.1
        center_rear.scale.y = 0.1
        center_rear.scale.z = 1.0
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
        center.pose.position.z = -0.5
        center.scale.x = 0.1
        center.scale.y = 0.1
        center.scale.z = 1.0
        center.color.b = 1.0
        center.color.a = 1.0

        # Build message for state.
        msg = MarkerArray()
        msg.markers.append(center_rear)
        msg.markers.append(center)

        pub.publish(msg)
