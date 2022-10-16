from time import sleep
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

class State(Enum):
    CHILLING = auto()
    BRICK_PUBLISHED = auto()

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
        self.reachable = False
        # assert all values greater than 0

        self.current = Point(x=0.0,y=0.0,z=self.platform_height)
        self.goal_sub = self.create_subscription(Point, "goal_message", self.goal_move_callback, 1)
        self.current_pos = Pose(x=0.0,y=0.0,theta=0.0,linear_velocity=0.0,angular_velocity=0.0)
        self.pos_or_subscriber = self.create_subscription(Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.reachable_pub = self.create_publisher(Marker, "/text_marker", 10)

        self.text_reachable = Marker(type=9)        

    def timer_callback(self):
        if self.state == State.BRICK_PUBLISHED and self.reachable == False:
            self.reachable_pub.publish(self.text_reachable)
        return
    
    def goal_move_callback(self, msg):
        goal = msg
        if (self.platform_height-goal.z) >= 0:
            t_req = math.sqrt(2*(self.platform_height-goal.z)/self.g)
        else:
            t_req = 0.0
        distance_goal = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        height_goal = goal.z - self.platform_height
        print(height_goal)
        self.state = State.BRICK_PUBLISHED
        if distance_goal/self.max_velocity <= t_req and height_goal >= .5*self.g*t_req**2:
            self.reachable = True
        else:    
            self.text_reachable.header.frame_id = "platform_tilt"
            self.text_reachable.header.stamp = self.get_clock().now().to_msg()
            self.text_reachable.action = 0
            self.text_reachable.scale.z = self.wheel_radius*2.5
            self.text_reachable.id = 10
            self.text_reachable.pose._position.z = self.wheel_radius
            self.text_reachable.text = "Unreachable"
            self.text_reachable.lifetime = Duration(sec=3.0,nanosec=0)
            self.text_reachable.color.a = 1.0
        return
    
    def pos_or_callback(self,msg):
        """Called by self.pos_or_subscriber
        Subscribes to pose, and updates current point (x,y) with pose (x,y)
        """
        self.current_pos.x = msg.x
        self.current_pos.y = msg.y
        return


def main(args=None):
    """Create a waypoint node and spin once"""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()