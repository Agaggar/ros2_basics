from time import sleep
import rclpy, math
from rclpy.node import Node
import rclpy.time
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
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
    CHILLING = auto()
    BRICK_FALLING = auto()
    CAUGHT = auto()
    UNCATCHABLE = auto()

class Catcher(Node):

    def __init__(self):
        super().__init__("catcher")
        self.frequency = 250.0
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.state = State.CHILLING
        self.declare_parameter("gravity", 9.8, ParameterDescriptor(description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("wheel_radius", 0.5, ParameterDescriptor(description="Wheel radius"))
        self.declare_parameter("platform_height", 0.6, ParameterDescriptor(description="height of platform. MUST BE >=3.5*WHEEL_RADIUS"))
        self.declare_parameter("max_velocity", 0.22, ParameterDescriptor(description="max linear velocity"))
        self.g = self.get_parameter("gravity").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.goal_pub = self.create_publisher(Point, "goal_message", 1)
        # assert all values greater than 0
        self.goal = None
        self.goal_initial = None
        self.t_req = 0.0
        self.prev_brick_z1 = None
        self.prev_brick_z2 = None
        self.current = Point(x=0.0, y=0.0, z=self.platform_height)
        self.current_pos = Pose(x=0.0, y=0.0, theta=0.0, linear_velocity=0.0, angular_velocity=0.0)
        self.brick_place = self.create_service(Place, "brick_place", self.place_callback)
        self.pos_or_subscriber = self.create_subscription(Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.reachable_pub = self.create_publisher(Marker, "/text_marker", 10)
        self.tilt_pub = self.create_publisher(Tilt, "tilt", 5)
        self.theta_tilt_default = math.pi/6

        self.text_reachable = Marker(type=9)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)    

    def timer_callback(self):
        print(self.state)
        try:
            self.world_brick = self.tf_buffer.lookup_transform(
                "world", "brick", rclpy.time.Time())
        except:
            # print("not published yet")
            return
        if self.world_brick:
            # print(self.world_brick.transform.translation.z)
            if self.prev_brick_z1 is None:
                self.prev_brick_z1 = self.world_brick.transform.translation.z
            else:
                if (self.prev_brick_z1 - self.world_brick.transform.translation.z) > 0.0:
                    if self.prev_brick_z2 is None:
                        self.prev_brick_z2 = self.world_brick.transform.translation.z
                    else:
                        if (self.prev_brick_z2 - self.world_brick.transform.translation.z) > 0.0:
                            self.state = State.BRICK_FALLING
                        else:
                            self.prev_brick_z1 = self.world_brick.transform.translation.z
                            self.prev_brick_z2 = None
                else:
                    self.prev_brick_z1 = self.world_brick.transform.translation.z
            if self.state == State.CAUGHT:
                self.prev_brick_z1 = self.goal_initial.z
                self.prev_brick_z2 = None
                try:
                    self.odom_brick = self.tf_buffer.lookup_transform(
                        "odom", "brick", rclpy.time.Time())
                except:
                    print("not published yet")
                    return
                if self.odom_brick:
                    if (self.odom_brick.transform.translation.x <= self.max_velocity/10.0) and (
                            self.odom_brick.transform.translation.y <= self.max_velocity/10.0):
                        self.tilt_pub.publish(Tilt(angle=self.theta_tilt_default))
                    if abs(self.odom_brick.transform.translation.z - self.goal_initial.z) <= 0.01:
                        # self.goal_pub.publish(self.goal_initial)
                        self.state = State.CHILLING
        print(self.prev_brick_z1, self.prev_brick_z2, self.world_brick.transform.translation.z)
        if self.state == State.BRICK_FALLING:
            self.check_goal()
        return
    
    def check_goal(self):
        goal = self.world_brick.transform.translation
        height_goal = goal.z - self.platform_height
        if height_goal >= 0:
            self.t_req = math.sqrt(2*(goal.z-self.platform_height)/self.g)
        else:
            self.t_req = 0.0
        distance_goal = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        if distance_goal/self.max_velocity <= self.t_req and self.state == State.BRICK_FALLING:
            self.reachable = True
            self.goal = Point(x=goal.x, y=goal.y, z=goal.z)
            if self.goal_initial is None:
                self.goal_initial = self.goal
            self.goal_pub.publish(self.goal)
            # if height_goal <= (1.5*self.wheel_radius + 0.05): # distance between origins
            #     self.state = State.CAUGHT
        else:
            self.state = State.UNCATCHABLE
        if self.state == State.UNCATCHABLE:
            self.text_reachable.header.frame_id = "platform_tilt"
            self.text_reachable.header.stamp = self.get_clock().now().to_msg()
            self.text_reachable.action = 0
            self.text_reachable.scale.z = self.wheel_radius*2.5
            self.text_reachable.id = 10
            self.text_reachable.pose.position.z = self.wheel_radius
            self.text_reachable.text = "Unreachable"
            self.text_reachable.lifetime = Duration(sec=3, nanosec=0)
            self.text_reachable.color.a = 1.0
            self.reachable_pub.publish(self.text_reachable)
        if distance_goal <= self.max_velocity/10.0:
            self.state = State.CAUGHT
            self.prev_brick_z1 = None
            self.prev_brick_z2 = None
        return

    def pos_or_callback(self, msg):
        """Called by self.pos_or_subscriber
        Subscribes to pose, and updates current point (x,y) with pose (x,y)
        """
        self.current_pos = msg
        return

    def place_callback(self, request, response):
        self.brick_place_initial = Point(x=request.brick_x, y=request.brick_y, z=request.brick_z)
        return response


def main(args=None):
    """Create a waypoint node and spin once"""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()