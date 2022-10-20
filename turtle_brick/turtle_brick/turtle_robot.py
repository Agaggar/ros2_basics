"""This node is in charge of moving the robot to and from the brick's x-y position.

PUBLISHERS:
  + publishes to: "turtle1/cmd_vel", type: geometry_msgs/msg/Twist - publishes a twist of the
    robot's current velocity
  + publishes to: "odom", type: nav_msgs/msg/Odometry - publishes the odometry heading of the
    robot (direct copy of cmd_vel)
  + publishes to: "joint_states", type: sensor_msg/msgs/JointState - publishes each joint's
    angles for visual effects and to connect to the transform tree
  + publishes to: "tf_static", type: geomtery_msgs/msg/TransformStamped - uses a 
    StaticTransformBroadcaster to publish static frames
  + publishes to: "tf", type: geomtery_msgs/msg/TransformStamped - uses a 
    TransformBroadcaster to publish frames

SUBSCRIBERS:
  + subscribes to: "turtle1/pose", type: turtlesim/msg/Pose - allows node to know where
    the turtle is at any given time, and therefore, knows where the robot is
  + subscribes to: "tilt", type: turtle_brick_interfaces/msg/Tilt - allows node to know how much
    to tilt the platform

SERVICES:
  + none

PARAMETERS:
  + name: gravity, type: float - acceleration due to gravity, as defined in config/turtle.yaml
  + name: wheel_radius, type: float - radius of wheel, as defined in config/turtle.yaml
  + name: platform_height, type: float - robot's platform height, as defined in config/turtle.yaml
          note that the platform_height must be >= 7*wheel_radius for robot geometry to be sensible
  + name: max_velocity, type: float - robot's maximum linear velocity, as defined in 
          config/turtle.yaml
"""

import rclpy
import math
import rclpy.time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from geometry_msgs.msg import Point, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from .quaternion import angle_axis_to_quaternion
from turtle_brick_interfaces.msg import Tilt

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState


class State(Enum):
    """Different possible states of the system.
    Determines what the main timer function should be doing on each iteration.
    """
    MOVING = auto()
    STOPPED = auto()
    CAUGHT = auto()
    WAITING = auto()
    TILT = auto()


class TurtleRobot(Node):
    """Creates robot by broadcasting frames, and controls robot's motions"""

    def __init__(self):
        """Initialize class variables."""
        super().__init__('turtle_node')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        self.pos_or_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.tilt_sub = self.create_subscription(Tilt, "tilt", self.tilt_callback, 5)
        self.vel_publisher = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.goal_sub = self.create_subscription(
            Point, "goal_message", self.goal_move_callback, 1)
        self.joint_state_pub = self.create_publisher(
            JointState, "joint_states", 10)
        self.current_pos = Pose(
            x=0.0,
            y=0.0,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0)
        self.current_twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                   angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.initial_spawn = False
        self.spawn_pos = Pose(
            x=5.5444,
            y=5.5444,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0)
        self.state = State.STOPPED
        self.goal = Point(x=0.0, y=0.0, z=0.0)
        self.goal_theta = 0.0
        self.tilt_angle = math.pi / 6

        self.declare_parameter("gravity", 9.8, ParameterDescriptor(
            description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(
            description="Wheel radius"))
        self.declare_parameter("platform_height", 6.0, ParameterDescriptor(
            description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(
            description="max linear velocity"))
        self.g = self.get_parameter(
            "gravity").get_parameter_value().double_value
        if self.g < 0:
            print("Acceleration due to gravity must be positive! Correcting...")
            self.g = abs(self.g)
        if self.g == 0:
            print("Acceleration due to gravity can't be 0! Defaulting...")
            self.g = 9.8
        self.wheel_radius = self.get_parameter(
            "wheel_radius").get_parameter_value().double_value
        if self.wheel_radius < 0:
            print("Wheel radius must be positive! Correcting...")
            self.wheel_radius = abs(self.wheel_radius)
        if self.wheel_radius == 0:
            print("Wheel radius can't be 0! Defaulting...")
            self.wheel_radius = 0.5
        self.platform_height = self.get_parameter(
            "platform_height").get_parameter_value().double_value
        if self.platform_height < 0:
            print("Platform height must be positive! Correcting...")
            self.platform_height = abs(self.platform_height)
        if self.platform_height < 7*self.wheel_radius:
            print("The platform height must be >=7*wheel_radius! Correcting platform height")
            self.platform_height = 7*self.wheel_radius
        self.max_velocity = self.get_parameter(
            "max_velocity").get_parameter_value().double_value
        if self.max_velocity < 0:
            print("Max velocity must be > 0 as defined in kinematic equations. Correcting...")
            self.max_velocity = abs(self.max_velocity)
        if self.max_velocity == 0:
            print("Max velocity can't be 0! Defaulting...")
            self.max_velocity = 2.2
        if self.max_velocity / 10.0 > 0.1:
            self.tolerance = 0.1
        else:
            self.tolerance = 0.1

        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = "world"
        world_base_tf.child_frame_id = "odom"
        world_base_tf.transform.translation.x = self.spawn_pos.x
        world_base_tf.transform.translation.y = self.spawn_pos.y
        world_base_tf.transform.translation.z = 0.0
        self.static_broadcaster.sendTransform(world_base_tf)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.js = JointState()
        self.js.name = ['wheel_stem', 'stem_base', 'platform_x', 'base_platform_fixed']
        self.js.position = [0.0, 0.0, 0.0, 0.0]
        self.tmr = self.create_timer(1 / 100.0, self.timer_callback)
        self.time = 0.0
        self.odom_base = TransformStamped()
        self.odom_base.header.frame_id = "odom"
        self.odom_base.child_frame_id = "base_link"

    def twist_to_odom(self, conv_twist):
        """Converts a geometry_msgs/msg/Twist to a nav_msgs/msg/Odometry type with no covariance
        Parameters: conv_twist, type geometry_msgs/msg/Twist
        Returns: type nav_msgs/msg/Odometry
        """
        head = Header()
        head.stamp = self.get_clock().now().to_msg()
        head.frame_id = "odom"
        pos = PoseWithCovariance()
        pos.pose.position = Point(
            x=self.current_pos.x,
            y=self.current_pos.y,
            z=0.0)
        pos.pose.orientation = angle_axis_to_quaternion(
            self.goal_theta, [0, 0, 1.0])
        twis = TwistWithCovariance()
        twis.twist = conv_twist
        return Odometry(
            header=head,
            child_frame_id="base_link",
            pose=pos,
            twist=twis)

    def timer_callback(self):
        """Checks different states to determine which function to call"""
        self.odom_base.header.stamp = self.get_clock().now().to_msg()
        self.odom_base.transform.translation.x = self.current_pos.x - self.spawn_pos.x
        self.odom_base.transform.translation.y = self.current_pos.y - self.spawn_pos.y
        self.broadcaster.sendTransform(self.odom_base)

        self.vel_publisher.publish(self.current_twist)
        self.odom_pub.publish(self.twist_to_odom(self.current_twist))
        self.js.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(self.js)

        if self.state == State.STOPPED:
            self.time = 0
            self.js.position = [0.0, 0.0, 0.0, 0.0]
        if self.state == State.MOVING:
            self.time += 1 / 100.0
            self.js.position[0] = self.max_velocity / self.wheel_radius * self.time
            self.js.position[1] = self.goal_theta
            self.move(self.goal)
        if self.state == State.WAITING:
            self.js.position = [0.0, 0.0, 0.0, 0.0]
            self.time = 0
            try:
                self.platform_brick = self.tf_buffer.lookup_transform(
                    "platform_tilt", "brick", rclpy.time.Time())
            except BaseException:
                # print("not published yet")
                return
            if self.platform_brick:
                if self.platform_brick.transform.translation.z <= self.wheel_radius * 3:
                    self.state = State.CAUGHT
        if self.state == State.CAUGHT:
            self.back_to_center()
            self.js.position[1] = self.goal_theta
            self.js.position[0] = self.max_velocity / self.wheel_radius * self.time
        if self.state == State.TILT:
            self.time += 1 / 100.0
            self.js.position = [0.0, 0.0, self.tilt_angle, 0.0]
            self.current_twist = Twist(
                linear=Vector3(
                    x=0.0, y=0.0, z=0.0), angular=Vector3(
                    x=0.0, y=0.0, z=0.0))
            t_req = math.sqrt(5 * self.wheel_radius * 2.0 / self.g / math.cos(self.tilt_angle))
            if self.time >= t_req:
                self.state = State.STOPPED

    def pos_or_callback(self, msg):
        """Subscribes to "/turtle1/pose", and updates current pose with pose
        msg is of type turtlesim/msg/Pose
        """
        if (self.initial_spawn is False):
            self.spawn_pos = msg
            self.initial_spawn = True
        self.current_pos = msg

    def goal_move_callback(self, msg):
        """Subscribes to "goal_message", with message type geometry_msgs/msg/Point"""
        if msg is not None and self.state == State.STOPPED:
            self.state = State.MOVING
        self.goal = msg
        return

    def move(self, goal):
        """Called when self.state == State.MOVING. Calculates heading to goal and moves to goal at
        max velocity. Switches to State.WAITING if at brick position waiting for brick to fall.
        Parameter: goal, type geometry_msg/msgs/Point
        """
        self.goal_theta = math.atan2(
            (goal.y - self.current_pos.y),
            (goal.x - self.current_pos.x))
        goal_distance = math.sqrt(
            (goal.y - self.current_pos.y)**2 + (goal.x - self.current_pos.x)**2)
        if goal_distance <= self.tolerance:
            self.current_twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                       angular=Vector3(x=0.0, y=0.0, z=0.0))
            self.state = State.WAITING
        elif goal_distance > self.tolerance:
            self.current_twist.linear = Vector3(
                x=self.max_velocity *
                math.cos(
                    self.goal_theta),
                y=self.max_velocity *
                math.sin(
                    self.goal_theta),
                z=0.0)
            self.state = State.MOVING

    def tilt_callback(self, msg):
        """Subscribed to "tilt". Updates angle with msg.angle to determine angle to tilt for
        robot's platform.
        Parameter: msg, type turtle_brick_interfaces/msg/Tilt
        """
        self.tilt_angle = msg.angle

    def back_to_center(self):
        """Called when self.state == State.CAUGHT; controls robot to go back to the home
        position. Very similar to move function, with subtle changes in switching states.
        """
        goal = Point(x=self.spawn_pos.x, y=self.spawn_pos.y, z=0.0)
        self.goal_theta = math.atan2(
            (goal.y - self.current_pos.y),
            (goal.x - self.current_pos.x))
        goal_distance = math.sqrt(
            (goal.y - self.current_pos.y)**2 + (goal.x - self.current_pos.x)**2)
        if goal_distance > self.tolerance:
            self.current_twist.linear = Vector3(
                x=self.max_velocity *
                math.cos(
                    self.goal_theta),
                y=self.max_velocity *
                math.sin(
                    self.goal_theta),
                z=0.0)
        if goal_distance <= self.tolerance:
            self.current_twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                       angular=Vector3(x=0.0, y=0.0, z=0.0))
            self.time = 0.0
            self.state = State.TILT


def main(args=None):
    """Create a turtle_robot node and spin."""
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
