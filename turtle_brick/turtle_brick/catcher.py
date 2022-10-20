"""This node is in charge of recognizing when the brick is falling, and whether the robot can move
fast enough in time to catch the brick or not.

PUBLISHERS:
  + publishes to: "text_marker", type: Marker - displays text "Unreachable" if brick can't be
    caught.
  + publishes to: "tilt", type: turtle_brick_interfaces/msg/Tilt - publishes the tilt angle for
    the platform.
  + publishes to: "goal_message", type: geometry_msg/msg/Point - publishes the goal of where the
    brick will fall IF the brick is catchable.

SUBSCRIBERS:
  + subscribes to: "turtle1/pose", type: turtlesim/msg/Pose - allows catcher node to know where
    the turtle is at any given time, and therefore, knows where the robot is.

SERVICES:
  + topic name: "brick_place" type: turtle_brick_interfaces/srv/Place - visually does nothing, but
    needed by the node to know where the brick is initially placed and determine whether it is
    falling or not.

PARAMETERS:
  + name: gravity, type: float - acceleration due to gravity, as defined in config/turtle.yaml
  + name: wheel_radius, type: float - radius of wheel, as defined in config/turtle.yaml
  + name: platform_height, type: float - robot's platform height, as defined in config/turtle.yaml
          note that the platform_height must be >= 7*wheel_radius for robot geometry to be sensible
"""

import rclpy
import math
from rclpy.node import Node
import rclpy.time
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from visualization_msgs.msg import Marker
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from turtle_brick_interfaces.srv import Place
from turtle_brick_interfaces.msg import Tilt
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    """Different possible states of the system.
    Determines what the main timer function should be doing on each iteration.
    """
    CHILLING = auto()
    BRICK_FALLING = auto()
    CAUGHT = auto()
    UNCATCHABLE = auto()


class Catcher(Node):
    """Determines whether a falling brick is catchable or not.
    If catchable, publish the goal to "goal_message"
    """

    def __init__(self):
        """Initialize class variables."""
        super().__init__("catcher")
        self.frequency = 250.0
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)
        self.state = State.CHILLING

        self.declare_parameter("gravity", 9.8, ParameterDescriptor(
            description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter(
            "wheel_radius", 0.5, ParameterDescriptor(
                description="Wheel radius"))
        self.declare_parameter("platform_height", 6.0, ParameterDescriptor(
            description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter(
            "max_velocity", 0.22, ParameterDescriptor(
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

        self.goal = None
        self.goal_initial = None
        self.distance_goal = 0.0
        self.t_req = 0.0
        self.prev_brick_z1 = None
        self.prev_brick_z2 = None
        self.current = Point(x=0.0, y=0.0, z=self.platform_height)
        self.current_pos = Pose(
            x=0.0,
            y=0.0,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0)
        self.text_reachable = Marker(type=9)
        self.text_count = 0
        self.theta_tilt_default = math.pi / 6
        if self.max_velocity / 10.0 > 0.1:
            self.tolerance = 0.1
        else:
            self.tolerance = 0.1

        self.brick_place = self.create_service(
            Place, "brick_place", self.place_callback)
        self.pos_or_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.reachable_pub = self.create_publisher(Marker, "text_marker", 10)
        self.tilt_pub = self.create_publisher(Tilt, "tilt", 5)
        self.goal_pub = self.create_publisher(Point, "goal_message", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        """Checks different states to determine which function to call"""
        try:
            self.world_brick = self.tf_buffer.lookup_transform(
                "world", "brick", rclpy.time.Time())
        except BaseException:
            # print("not published yet")
            return
        if self.world_brick:
            self.is_falling()
            if self.state == State.CAUGHT:
                self.prev_brick_z1 = self.goal_initial.z
                self.prev_brick_z2 = None
                self.tilt_brick()
            if self.state == State.BRICK_FALLING:
                self.check_goal()
        # print(self.state)

    def is_falling(self):
        """Called only if the brick has been published. If yes, check if the brick is falling.
        Brick is falling if it's z-height off the ground changes for two CONSECUTIVE frames.
        If brick is falling, change the state to BRICK_FALLING. Otherwise, reset the previous two
        frames.
        """
        if self.prev_brick_z1 is None:
            self.prev_brick_z1 = self.world_brick.transform.translation.z
        else:
            if (self.prev_brick_z1 -
                    self.world_brick.transform.translation.z) > 0.0:
                if self.prev_brick_z2 is None:
                    self.prev_brick_z2 = self.world_brick.transform.translation.z
                else:
                    if (self.prev_brick_z2 -
                            self.world_brick.transform.translation.z) > 0.0:
                        self.state = State.BRICK_FALLING
                    else:
                        self.prev_brick_z1 = self.world_brick.transform.translation.z
                        self.prev_brick_z2 = None
            else:
                self.prev_brick_z1 = self.world_brick.transform.translation.z
                self.prev_brick_z2 = None

    def check_goal(self):
        """Called if state is BRICK_FALLING. Checks to make sure that both the height and
        distance of the brick to the robot are within the kinematic requirements to catch the
        brick in time, given the max velocity. If catchable, publish the goal. If not, switch
        state to UNCATCHABLE. If the distance to goal decreases to "0" tolerance, change state
        to CAUGHT.
        """
        goal = self.world_brick.transform.translation
        height_goal = goal.z - self.platform_height
        self.distance_goal = math.sqrt(
            (goal.y - self.current_pos.y)**2 + (goal.x - self.current_pos.x)**2)
        catchable = catch_dist(height_goal, self.distance_goal, self.g, self.max_velocity)
        if goal.x >= 11 or goal.y >= 11:
            catchable = False
        if catchable is True and self.state == State.BRICK_FALLING:
            self.goal = Point(x=goal.x, y=goal.y, z=goal.z)
            if self.goal_initial is None:
                self.goal_initial = self.goal
            self.goal_pub.publish(self.goal)
        elif catchable is False and self.state == State.BRICK_FALLING:
            self.state = State.UNCATCHABLE
            self.uncatchable_pub()
        if self.distance_goal <= self.tolerance:
            self.state = State.CAUGHT
            self.prev_brick_z1 = None
            self.prev_brick_z2 = None
        return

    def uncatchable_pub(self):
        """Called within check_goal if state is UNCATCHABLE and the brick is in a "placed"
        position. Create the uncatchable text marker, and switch state to original state.
        Note that this means the robot should never have moved in the first place, and should
        be in its original position.
        """
        if self.prev_brick_z2 is not None:
            self.text_reachable.header.frame_id = "platform_tilt"
            self.text_reachable.header.stamp = self.get_clock().now().to_msg()
            self.text_reachable.action = 0
            self.text_reachable.scale.z = self.wheel_radius * 2.5
            self.text_reachable.id = 10
            self.text_reachable.pose.position.z = self.wheel_radius
            self.text_reachable.text = "Unreachable"
            self.text_reachable.lifetime = Duration(sec=3)
            self.text_reachable.color.r = 230.0 / 255.0
            self.text_reachable.color.g = 217.0 / 255.0
            self.text_reachable.color.b = 200.0 / 255.0
            self.text_reachable.color.a = 1.0
            if self.text_count == 0:
                print('hello')
                self.reachable_pub.publish(self.text_reachable)
                self.text_count += 1
            self.state = State.CHILLING

    def tilt_brick(self):
        """Called if the state is CAUGHT (note that the robot is back to its starting position
        when this is called). Publishes the tilt angle (which can be changed by changing the
        class variable self.theta_tilt_default). Once the brick resets to it's original
        position, the state resets to the original state, CHILLING.
        """
        try:
            self.odom_brick = self.tf_buffer.lookup_transform(
                "odom", "brick", rclpy.time.Time())
        except BaseException:
            print("not published yet")
            return
        if self.odom_brick:
            if (self.odom_brick.transform.translation.x <= self.tolerance) and (
                    self.odom_brick.transform.translation.y <= self.tolerance):
                self.tilt_pub.publish(
                    Tilt(angle=self.theta_tilt_default))
            if abs(
                    self.odom_brick.transform.translation.z -
                    self.goal_initial.z) <= 0.01:
                self.state = State.CHILLING

    def pos_or_callback(self, msg):
        """Called by self.pos_or_subscriber.
        Subscribes to "/turtle1/pose", and updates current point (x,y) with pose (x,y)
        """
        self.current_pos = msg
        return

    def place_callback(self, request, response):
        """Called by self.place_brick. Reads the service input to change the class variable
        brick_place_initial to the value determined by the service.
        Service for "brick_place".
        Request of type Place
        Response of type std_srvs.srv.Empty
        """
        self.brick_place_initial = Point(
            x=request.brick_x,
            y=request.brick_y,
            z=request.brick_z)
        self.text_count = 0
        self.prev_brick_z2 = None
        return response


def catch_dist(height, distance, g, max_vel):
    """Called by check goal, returns True or False depending on kinematics.
    Parameters:
    - height (float) - height from which brick falls
    - distance (float) - x-y distance to brick
    - g (float) - acceleration due to gravity
    - max_vel (float) - max_velocity of the robot
    Returns:
    - boolean True or False
    """
    if height <= 0 or distance <= 0 or g <= 0 or max_vel <= 0:
        return False
    if height >= 0:
        t = math.sqrt(2 * height / g)
    else:
        t = 0.0
    return distance / max_vel <= t


def main(args=None):
    """Create a catcher node and spin once."""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
