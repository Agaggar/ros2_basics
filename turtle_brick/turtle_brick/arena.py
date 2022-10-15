from email.header import Header
# import re
from turtle import Turtle, reset
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from turtle_brick_interfaces.srv import Place
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
import time
from math import pi
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Quaternion
from builtin_interfaces.msg import Duration

from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class State(Enum):
    RUNNING = auto()
    PLACE_BRICK = auto()
    DROP_BRICK = auto()

class Arena(Node):
    def __init__(self):
        super().__init__('arena')
        self.count = 0
        self.frequency = 250.0
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.marker_pub = self.create_publisher(Marker, "/wall_marker", 10)
        self.brick_pub = self.create_publisher(Marker, "/brick_marker", 10)
        
        self.brick_place = self.create_service(Place, "brick_place", self.place_callback)
        # self.brick_place_client = self.create_client(Place, "brick_place")
        self.brick_drop = self.create_service(Empty, "brick_drop", self.drop_callback)
        # self.brick_place_client = self.create_client(Place, "brick_place")
        self.state = State.RUNNING
        self.broadcaster = TransformBroadcaster(self)
        self.time = 0.0
        self.brick_z_initial = 0.0
        self.declare_parameter("gravity", 9.8, ParameterDescriptor(description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(description="Wheel radius"))
        self.declare_parameter("platform_height", 0.6, ParameterDescriptor(
            description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(description="max linear velocity"))
        self.g = self.get_parameter("gravity").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        # assert all values greater than 0

        self.goal_pub = self.create_publisher(Point, "goal_message",1)
        
        self.marker_walls_border = Marker()
        self.marker_walls_border.header.frame_id = "world"
        self.marker_walls_border.header.stamp = self.get_clock().now().to_msg()
        self.marker_walls_border.type = 6
        self.marker_walls_border.id = 0
        self.marker_walls_border.action = 0
        self.marker_walls_border.scale.x = 1.0
        self.marker_walls_border.scale.y = 1.0
        self.marker_walls_border.scale.z = 2.0
        self.marker_walls_border.color.b = 1.0
        self.marker_walls_border.color.a = 1.0

        self.points = []
        self.points.append(Point(x=0.0, y=0.0, z=self.marker_walls_border.scale.z/2))
        for x in range(1,12):
            self.points.append(Point(x=float(x), y=0.0, z=self.marker_walls_border.scale.z/2))
            self.points.append(Point(x=float(x), y=11.0, z=self.marker_walls_border.scale.z/2.0))
        for y in range(1,12):
            self.points.append((Point(x=0.0, y=float(y), z=self.marker_walls_border.scale.z/2)))
            self.points.append((Point(x=11.0, y=float(y), z=self.marker_walls_border.scale.z/2)))

        self.marker_walls_border.points = self.points

        self.marker_brick = Marker(type=1)
        self.marker_brick.header.frame_id = "world"
        self.marker_brick.header.stamp = self.get_clock().now().to_msg()
        self.marker_brick.color.r = 132.0/255.0
        self.marker_brick.color.g = 31.0/255.0
        self.marker_brick.color.b = 39.0/255.0
        self.marker_brick.action = 0
        self.marker_brick.scale.x = 1.0
        self.marker_brick.scale.y = 2.0
        self.marker_brick.scale.z = 1.0
    
    def timer_callback(self):
        if (self.count%25) == 0:
            self.marker_pub.publish(self.marker_walls_border)
        self.odom_brick = TransformStamped()
        self.odom_brick.header.stamp = self.get_clock().now().to_msg()
        self.odom_brick.header.frame_id = "world"
        self.odom_brick.child_frame_id = "brick"
        self.odom_brick.transform.translation.x = self.marker_brick.pose.position.x
        self.odom_brick.transform.translation.y = self.marker_brick.pose.position.y
        self.odom_brick.transform.translation.z = self.marker_brick.pose.position.z
        self.broadcaster.sendTransform(self.odom_brick)
        
        if self.state != State.RUNNING:
            self.brick_pub.publish(self.marker_brick)
        if self.state == State.DROP_BRICK:
            self.time = self.time + 1/self.frequency
            self.marker_brick.pose.position.z = self.brick_z_initial - 0.5*self.g*self.time**2
            # if self.marker_brick.pose.position.z == self.platform.z and (abs(self.marker_brick.pose.position.x-self.platform.x)<=self.wheel_radius) and (abs(self.marker_brick.pose.position.y-self.platform.y)<=self.wheel_radius)
            if self.marker_brick.pose.position.z <= self.marker_brick.scale.z/2.0:
                self.state = State.RUNNING
                self.brick_z_initial=0.0
        self.count +=1

    def place_callback(self, request, response):
        self.state = State.PLACE_BRICK
        self.marker_brick.pose.position.x = request.brick_x
        self.marker_brick.pose.position.y = request.brick_y
        self.marker_brick.pose.position.z = request.brick_z
        self.marker_brick.color.a = 1.0
        self.time = 0.0
        self.brick_z_initial = request.brick_z
        return response
    
    def drop_callback(self,request,response):
        self.goal_pub.publish(
            Point(x=self.marker_brick.pose.position.x, 
            y=self.marker_brick.pose.position.y, 
            z=self.brick_z_initial))
        if self.state == State.PLACE_BRICK:
            self.state = State.DROP_BRICK
        else:
            print("Place brick first!")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()