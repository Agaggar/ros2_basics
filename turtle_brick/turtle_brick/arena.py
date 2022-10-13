from email.header import Header
from turtle import Turtle, reset
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from turtle_brick_interfaces.srv import Place
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
import time
from math import pi
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Point, Quaternion
from builtin_interfaces.msg import Duration

from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class State(Enum):
    RUNNING = auto()
    DROP_BRICK = auto()

class Arena(Node):
    def __init__(self):
        super().__init__('arena')
        self.count = 0
        self.timer = self.create_timer(1/250.0, self.timer_callback)
        self.marker_pub = self.create_publisher(Marker, "/wall_marker", 10)
        self.brick_pub = self.create_publisher(Marker, "/brick_marker", 10)
        # self.marker_pub = self.create_publisher(MarkerArray, "/wall_marker", 10)
        self.brick_place = self.create_service(Place, "brick_place", self.place_callback)
        self.brick_place_client = self.create_client(Place, "brick_place")
        self.state = State.RUNNING
        self.broadcaster = TransformBroadcaster(self)

    def create_wall(self):
        self.marker_walls_border = Marker()
        self.marker_walls_border.header.frame_id = "platform_fixed"
        self.marker_walls_border.header.stamp = self.get_clock().now().to_msg()
        self.marker_walls_border.type = 6
        self.marker_walls_border.id = 0
        self.marker_walls_border.action = 0
        self.marker_walls_border.scale.x = 1.0
        self.marker_walls_border.scale.y = 1.0
        self.marker_walls_border.scale.z = 1.0
        self.marker_walls_border.color.b = 1.0
        self.marker_walls_border.color.a = 1.0
        # self.marker_walls_border.lifetime = Duration(0,0)
        self.marker_walls_border.pose.position.x = 0.0
        self.marker_walls_border.pose.position.y = 0.0
        self.marker_walls_border.pose.position.z = 0.0
        self.marker_walls_border.pose.orientation.x = 0.0
        self.marker_walls_border.pose.orientation.y = 0.0
        self.marker_walls_border.pose.orientation.z = 0.0
        self.marker_walls_border.pose.orientation.w = 0.0
        self.points = []
        self.points.append(Point(x=0.0, y=0.0, z=0.0))
        for x in range(1,11):
            self.points.append(Point(x=float(x), y=0.0, z=0.0))
            self.points.append(Point(x=float(x), y=10.0, z=0.0))
        for y in range(1,11):
            self.points.append((Point(x=0.0, y=float(y), z=0.0)))
            self.points.append((Point(x=10.0, y=float(y), z=0.0)))

        self.marker_walls_border.points = self.points

        self.marker_brick = Marker(type=1)
        self.marker_brick.header.frame_id = "platform_fixed"
        self.marker_brick.header.stamp = self.get_clock().now().to_msg()
        self.marker_brick.color.r = 132.0/255.0
        self.marker_brick.color.g = 31.0/255.0
        self.marker_brick.color.b = 39.0/255.0
        self.marker_brick.color.a = 0.0
        self.marker_brick.action = 0
        self.marker_brick.scale.x = 1.0
        self.marker_brick.scale.y = 2.0
        self.marker_brick.scale.z = 1.0
        
        return self.marker_walls_border
    
    def timer_callback(self):
        if (self.count%25) == 0:
            self.marker_pub.publish(self.create_wall())
        self.world_brick = TransformStamped()
        self.world_brick.header.stamp = self.get_clock().now().to_msg()
        self.world_brick.header.frame_id = "world"
        self.world_brick.child_frame_id = "brick"
        # world_brick.transform.translation.x = float(self.dx)
        # world_brick.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])
        
        self.world_brick.transform.translation.x = self.marker_brick.pose.position.x
        self.world_brick.transform.translation.y = self.marker_brick.pose.position.y
        self.world_brick.transform.translation.z = self.marker_brick.pose.position.z
        self.broadcaster.sendTransform(self.world_brick)
        self.g = 9.81
        if self.state == State.DROP_BRICK:
            self.brick_pub.publish(self.marker_brick)
        self.count +=1

    def place_callback(self, request, response):
        self.state == State.DROP_BRICK
        
        self.marker_brick.pose.position.x = request.brick_x
        self.marker_brick.pose.position.y = request.brick_y
        self.marker_brick.pose.position.z = request.brick_z
        self.marker_brick.color.a = 1.0
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()