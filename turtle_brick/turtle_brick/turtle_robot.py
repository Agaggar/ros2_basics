from turtle import Turtle, position, reset
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from turtle_brick_interfaces.msg import Tilt
import time
from math import pi
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Point, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from .quaternion import angle_axis_to_quaternion

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class State(Enum):
    MOVING = auto()
    STOPPED = auto()
    CAUGHT = auto()
    TILT = auto()

class TurtleRobot(Node):
    """ Creates robot
    Static broadcasts: 
        - world. All other frames are descendants of this
        - odom. Fixed offset from world; denotes starting point of robot
    Broadcasts: 
        - brick. Frame of brick
        - base_link. Base of robot; other child frames defined by URDF
    """

    def __init__(self):
        super().__init__('turtle_node')
        # Static broadcasters publish on /tf_static. We will only need to publish this once
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        self.pos_or_subscriber = self.create_subscription(Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.goal_sub = self.create_subscription(Point, "goal_message", self.goal_move_callback, 1)
        self.current_pos = Pose(x=0.0,y=0.0,theta=0.0,linear_velocity=0.0,angular_velocity=0.0)
        self.current_twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        # change spawn_pos to be a parameter that's passed in
        self.spawn_pos = Pose(x=5.5444, y=5.5444, theta=0.0, linear_velocity=0.0,angular_velocity=0.0)
        self.odom_bool = False
        self.state = State.STOPPED
        self.goal = Point(x=0.0,y=0.0,z=0.0)
        self.goal_theta = 0.0

        self.declare_parameter("gravity", 9.8, ParameterDescriptor(description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(description="Wheel radius"))
        self.declare_parameter("platform_height", 0.6, ParameterDescriptor(description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(description="max linear velocity"))
        self.g = self.get_parameter("gravity").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value

        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = "world"
        world_base_tf.child_frame_id = "odom"
        world_base_tf.transform.translation.x = self.spawn_pos.x
        world_base_tf.transform.translation.y = self.spawn_pos.y
        world_base_tf.transform.translation.z = 0.0
        self.static_broadcaster.sendTransform(world_base_tf)

        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(1/100.0, self.timer_callback)
    
    def twist_to_odom(self, conv_twist):
        head = Header()
        head.stamp = self.get_clock().now().to_msg()
        head.frame_id = "odom"
        pos = PoseWithCovariance()
        pos.pose.position = Point(x=self.current_pos.x, y=self.current_pos.y, z=0.0)
        pos.pose.orientation = angle_axis_to_quaternion(self.goal_theta, [0, 0, 1.0])
        twis = TwistWithCovariance()
        twis.twist = conv_twist
        return Odometry(header=head,child_frame_id="base_link", pose=pos, twist=twis)

    def timer_callback(self): 
        self.odom_base = TransformStamped()
        self.odom_base.header.frame_id = "odom"
        self.odom_base.child_frame_id = "base_link"
        self.odom_base.header.stamp = self.get_clock().now().to_msg()
        self.odom_base.transform.translation.x = self.current_pos.x - self.spawn_pos.x # offset by half of wheel_length
        self.odom_base.transform.translation.y = self.current_pos.y - self.spawn_pos.y
        self.broadcaster.sendTransform(self.odom_base)

        if self.state == State.MOVING:
            self.move(self.goal)
        if self.state == State.STOPPED:
            self.vel_publisher.publish(self.current_twist)
        if self.state == State.CAUGHT:
            self.back_to_center()
        if self.state == State.TILT:
            self.current_twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.vel_publisher.publish(self.current_twist)
            
    
    def pos_or_callback(self, msg):
        """Called by self.pos_or_subscriber
        Subscribes to pose, and updates current with pose
        """
        if (self.spawn_pos == None):
            self.spawn_pos = msg
            self.odom_bool = True
        self.current_pos = msg
    
    def goal_move_callback(self, msg):
        if msg != None:
            self.state = State.MOVING
        self.goal = msg
        return
    
    def move(self, goal):
        self.goal_theta = math.atan2((goal.y-self.current_pos.y), (goal.x-self.current_pos.x))
        goal_distance = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        if goal_distance > self.max_velocity/10.0:
            self.current_twist.linear = Vector3(x = self.max_velocity*math.cos(self.goal_theta), y = self.max_velocity*math.sin(self.goal_theta), z = 0.0)
        if goal_distance <= self.max_velocity/10.0:
            self.current_twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.state = State.CAUGHT
        self.vel_publisher.publish(self.current_twist)
        self.odom_pub.publish(self.twist_to_odom(self.current_twist))
        # print(goal.y-self.current_pos.y)
    
    def back_to_center(self):
        self.goal_theta = pi + self.goal_theta
        goal = Point(x=self.spawn_pos.x, y=self.spawn_pos.y, z=0.0)
        # print(goal.y-self.current_pos.y)
        goal_distance = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        if goal_distance > self.max_velocity/10.0:
            # self.current_twist.angular.z = 0.0
            self.current_twist.linear = Vector3(x = self.max_velocity*math.cos(self.goal_theta), y = self.max_velocity*math.sin(self.goal_theta), z = 0.0)
        else:
            self.current_twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.state = State.TILT
        self.vel_publisher.publish(self.current_twist)
        self.odom_pub.publish(self.twist_to_odom(self.current_twist))
        
        return

    def tilt_callback(self,request,response):
        tilt_angle = request.angle
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()