from turtle import Turtle, position, reset
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
# from turtle_brick_interfaces.srv blah blah
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
import time
from math import pi
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

class State(Enum):
    MOVING = auto()
    STOPPED = auto()

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
        self.pos_or_subscriber = self.create_subscription(Pose, "turtlesim/turtle1/pose", self.pos_or_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, "turtlesim/turtle1/cmd_vel", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.goal_sub = self.create_subscription(Pose, "goal_message", self.goal_move_callback, 1)
        self.current_pos = Pose(x=0.0,y=0.0,theta=0.0,linear_velocity=0.0,angular_velocity=0.0)
        self.spawn_pos = None
        self.odom_bool = False
        self.state = State.STOPPED
        self.goal = Pose(x=0.0,y=0.0,theta=0.0,linear_velocity=0.0,angular_velocity=0.0)

        self.declare_parameter("gravity", 9.8, ParameterDescriptor(description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(description="Wheel radius"))
        self.declare_parameter("platform_height", 0.6, ParameterDescriptor(description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(description="max linear velocity"))
        self.g = self.get_parameter("gravity").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value

        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(1/100.0, self.timer_callback)

    def timer_callback(self):
        if self.odom_bool == True:
            world_base_tf = TransformStamped()
            world_base_tf.header.stamp = self.get_clock().now().to_msg()
            world_base_tf.header.frame_id = "world"
            world_base_tf.child_frame_id = "odom"
            world_base_tf.transform.translation.x = self.spawn_pos.x
            world_base_tf.transform.translation.y = self.spawn_pos.y
            world_base_tf.transform.translation.z = 0.0
            self.static_broadcaster.sendTransform(world_base_tf)
        
        self.odom_base = TransformStamped()
        self.odom_base.header.frame_id = "odom"
        self.odom_base.child_frame_id = "base_link"
        self.odom_base.header.stamp = self.get_clock().now().to_msg()
        self.odom_base.transform.translation.x = self.current_pos.x - self.spawn_pos.x # offset by half of wheel_length
        self.odom_base.transform.translation.y = self.current_pos.y - self.spawn_pos.y
        self.broadcaster.sendTransform(self.odom_base)

        if self.state == State.MOVING:
            self.move(self.goal)
    
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
        return
    
    def move(self, goal):
        goal_theta = math.atan2((goal.y-self.current_pos.y), (goal.x-self.current_pos.x))
        goal_distance = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        if goal_distance <= 0:
            self.vel_publisher.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
        if abs(goal.theta - goal_theta)>0.1:
            self.vel_publisher.publish(Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 3.0)))
        curr_twist = Twist(linear = Vector3(x = self.max_velocity, y = self.max_velocity, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()