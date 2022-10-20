"""
This file controls the visuals of the arena walls and the brick.
PUBLISHERS:
  + publishes to: "wall_marker", type: Marker - publishes a cube list, representing the walls.
  + publishes to: "brick_marker", type: Marker - publishes the brick visual.

SUBSCRIBERS:
  + subscribes to: "turtle1/pose", type: turtlesim/msg/Pose - allows node to know where
    the turtle is at any given time, and therefore, knows where the robot is.
  + subscribes to: "tilt", type: turtle_brick_interfaces/msg/Tilt - allows node to know how much
    to tilt the platform.

SERVICES:
  + name: "brick_place", type: turtle_brick_interfaces/srv/Place - places the brick to the
    position defined by the service.
  + name: "brick_drop", type: std_srvs/srv/Empty - drops the brick from the placed position.

PARAMETERS:
  + name: gravity, type: float - acceleration due to gravity, as defined in config/turtle.yaml
  + name: wheel_radius, type: float - radius of wheel, as defined in config/turtle.yaml
  + name: platform_height, type: float - robot's platform height, as defined in config/turtle.yaml
          note that the platform_height must be >= 7*wheel_radius for robot geometry to be sensible
  + name: max_velocity, type: float - robot's maximum linear velocity, as defined in
          config/turtle.yaml
"""

from enum import Enum, auto
import math
import rclpy
from rclpy.node import Node
import rclpy.time
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from turtle_brick_interfaces.srv import Place
from turtle_brick_interfaces.msg import Tilt
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    """ Current state of the system.
        Determines what timer and subfunctions should do in each state.
    """
    RUNNING = auto()
    PLACE_BRICK = auto()
    DROP_BRICK = auto()
    BRICK_PLATFORM = auto()
    TILTING_OFF = auto()
    TILT_ORIGINAL = auto()


class Arena(Node):
    """Publishes visuals of the brick and arena."""

    def __init__(self):
        super().__init__('arena')
        self.count = 0
        self.frequency = 250.0
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)
        self.marker_pub = self.create_publisher(Marker, "wall_marker", 10)
        self.brick_pub = self.create_publisher(Marker, "brick_marker", 10)
        self.brick_place = self.create_service(
            Place, "brick_place", self.place_callback)
        self.brick_drop = self.create_service(
            Empty, "brick_drop", self.drop_callback)
        self.tilt_sub = self.create_subscription(
            Tilt, "tilt", self.tilt_callback, 5)
        self.pos_or_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pos_or_callback, 10)

        self.state = State.RUNNING
        self.broadcaster = TransformBroadcaster(self)
        self.time = 0.0
        self.brick_z_initial = 0.0
        self.current_pos = Pose(
            x=0.0,
            y=0.0,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0)
        self.tilt_def = math.pi / 6
        self.declare_parameter("gravity", 9.8, ParameterDescriptor(
            description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(
            description="Wheel radius"))
        self.declare_parameter("platform_height", 6.0, ParameterDescriptor(
            description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(
            description="max linear velocity"))
        self.accel_g = self.get_parameter(
            "gravity").get_parameter_value().double_value
        if self.accel_g < 0:
            print("Acceleration due to gravity must be positive! Correcting...")
            self.accel_g = abs(self.accel_g)
        if self.accel_g == 0:
            print("Acceleration due to gravity can't be 0! Defaulting...")
            self.accel_g = 9.8
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

        self.marker_walls_border = Marker()
        self.marker_walls_border.header.frame_id = "world"
        self.marker_walls_border.header.stamp = self.get_clock().now().to_msg()
        self.marker_walls_border.type = 6
        self.marker_walls_border.id = 0
        self.marker_walls_border.action = 0
        self.marker_walls_border.scale.x = 1.0
        self.marker_walls_border.scale.y = 1.0
        self.marker_walls_border.scale.z = self.platform_height/2.0
        self.marker_walls_border.color.b = 1.0
        self.marker_walls_border.color.a = 1.0
        self.points = []
        self.points.append(
            Point(
                x=0.0,
                y=0.0,
                z=self.marker_walls_border.scale.z /
                2))
        for row in range(1, 12):
            self.points.append(
                Point(
                    x=float(row),
                    y=0.0,
                    z=self.marker_walls_border.scale.z /
                    2))
            self.points.append(
                Point(
                    x=float(row),
                    y=11.0,
                    z=self.marker_walls_border.scale.z /
                    2.0))
        for coln in range(1, 12):
            self.points.append(
                (Point(
                    x=0.0,
                    y=float(coln),
                    z=self.marker_walls_border.scale.z /
                    2)))
            self.points.append(
                (Point(
                    x=11.0,
                    y=float(coln),
                    z=self.marker_walls_border.scale.z /
                    2)))
        self.marker_walls_border.points = self.points
        self.platform_brick = None
        self.marker_brick = Marker(type=1)
        self.marker_brick.header.frame_id = "world"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.brick_place_initial = Point(x=0.0, y=0.0, z=0.0)
        self.world_brick = TransformStamped()

    def timer_callback(self):
        """Checks different states to determine which function to call."""
        if (self.count % 50) == 0:
            self.marker_pub.publish(self.marker_walls_border)
        if self.state != State.RUNNING:
            self.world_brick.header.stamp = self.get_clock().now().to_msg()
            self.world_brick.header.frame_id = "world"
            self.world_brick.child_frame_id = "brick"
            self.world_brick.transform.translation.x = self.marker_brick.pose.position.x
            self.world_brick.transform.translation.y = self.marker_brick.pose.position.y
            self.world_brick.transform.translation.z = self.marker_brick.pose.position.z
            self.broadcaster.sendTransform(self.world_brick)
            self.brick_pub.publish(self.marker_brick)

        if self.state == State.DROP_BRICK:
            try:
                self.platform_brick = self.tf_buffer.lookup_transform(
                    "platform_tilt", "brick", rclpy.time.Time())
            except BaseException:
                print("not published yet")
                return
            self.time = self.time + 1 / self.frequency
            self.marker_brick.pose.position.z = self.brick_z_initial - (
                0.5 * self.accel_g * self.time**2)
            if (abs(self.marker_brick.pose.position.z - self.marker_brick.scale.z / 2.0) <=
                self.platform_height) and (
                abs(self.platform_brick.transform.translation.x) <=
                    self.wheel_radius * 5) and (abs(
                        self.platform_brick.transform.translation.y) <=
                    self.wheel_radius * 5) and self.brick_place_initial.z >= self.platform_height:
                self.time = 0.0
                self.state = State.BRICK_PLATFORM
            if self.marker_brick.pose.position.z <= self.marker_brick.scale.z / 2.0:
                self.state = State.RUNNING
                self.brick_z_initial = 0.0
        if self.state == State.BRICK_PLATFORM:
            self.marker_brick.pose.position.x = self.current_pos.x
            self.marker_brick.pose.position.y = self.current_pos.y
            self.marker_brick.pose.position.z = self.platform_height + \
                self.marker_brick.scale.z / 2.0
            self.brick_y_initial = self.world_brick.transform.translation.y
            self.brick_z_initial = self.world_brick.transform.translation.z
            try:
                self.odom_brick = self.tf_buffer.lookup_transform(
                    "odom", "brick", rclpy.time.Time())
            except BaseException:
                print("not published yet")
                return
            if self.odom_brick:
                if (abs(self.odom_brick.transform.translation.x) <= self.max_velocity / 10.0) and (
                        abs(self.odom_brick.transform.translation.y) <= self.max_velocity / 10.0):
                    self.state = State.TILTING_OFF
        if self.state == State.TILTING_OFF:
            self.tilt_brick()
        if self.state == State.TILT_ORIGINAL:
            self.tilt_brick()
        self.count += 1

    def place_callback(self, request, response):
        """Reads the service input to publish the visual for the brick.
        Service for "brick_place".
        Keyword arguments:
        request -- type turtle_brick_interfaces/srv/Place
        response -- type std_srvs.srv.Empty
        """
        self.state = State.PLACE_BRICK
        self.marker_brick.header.stamp = self.get_clock().now().to_msg()
        self.marker_brick.color.r = 132.0 / 255.0
        self.marker_brick.color.g = 31.0 / 255.0
        self.marker_brick.color.b = 39.0 / 255.0
        self.marker_brick.action = 0
        self.marker_brick.scale.x = 2.0 * self.wheel_radius
        self.marker_brick.scale.y = 2.0 * self.marker_brick.scale.x
        self.marker_brick.scale.z = self.marker_brick.scale.x
        self.marker_brick.pose.position.x = request.brick_x
        self.marker_brick.pose.position.y = request.brick_y
        self.marker_brick.pose.position.z = request.brick_z
        self.marker_brick.color.a = 1.0
        self.time = 0.0
        self.brick_z_initial = request.brick_z
        self.brick_place_initial = Point(
            x=request.brick_x,
            y=request.brick_y,
            z=request.brick_z)
        return response

    def drop_callback(self, request, response):
        """Drops brick.
        Keyword arguments:
        request -- type std_srvs.srv.Empty
        response -- type std_srvs.srv.Empty
        """
        if self.state == State.PLACE_BRICK:
            self.state = State.DROP_BRICK
        else:
            print("Place brick first!")
        return response

    def pos_or_callback(self, msg):
        """Subscribes to "/turtle1/pose", and updates current pose with pose.
        Keyword arguments:
        msg -- type turtlesim/msg/Pose
        """
        self.current_pos = msg

    def tilt_callback(self, msg):
        """Subscribed to "tilt". Updates angle with msg.angle to determine angle to tilt for
        robot's platform.
        Keyword arguments:
        msg -- type turtle_brick_interfaces/msg/Tilt
        """
        self.tilt_def = msg.angle

    def tilt_brick(self):
        """Determines visuals of the brick while tilting."""
        t_req = math.sqrt(
            5 * self.wheel_radius *
            2.0 /
            self.accel_g /
            math.cos(
                self.tilt_def))
        self.time += 1 / self.frequency
        if self.state == State.TILTING_OFF:
            self.marker_brick.pose.orientation.x = self.tilt_def / 2
            self.world_brick.transform.rotation.x = self.tilt_def
            self.marker_brick.pose.position.y = (
                self.brick_y_initial - 0.5 * self.accel_g *
                math.cos(
                    self.tilt_def) *
                self.time**2)
            self.marker_brick.pose.position.z = (
                self.brick_z_initial -
                0.5 *
                self.accel_g *
                math.sin(
                    self.tilt_def) *
                self.time**2)
        if self.time >= t_req:
            self.brick_z_initial = self.marker_brick.pose.position.z
            self.state = State.TILT_ORIGINAL
        if self.state == State.TILT_ORIGINAL:
            self.marker_brick.pose.orientation.x = 0 * self.tilt_def
            self.world_brick.transform.rotation.x = 0 * self.tilt_def
            self.marker_brick.pose.position.x = self.brick_place_initial.x
            self.marker_brick.pose.position.y = self.brick_place_initial.y
            self.marker_brick.pose.position.z = self.brick_place_initial.z
            self.state = State.PLACE_BRICK
            self.brick_z_initial = self.brick_place_initial.z


def main(args=None):
    """Create an arena node and spin."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
