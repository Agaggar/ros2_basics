from time import sleep
import rclpy, math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Point

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
        self.pos_or_subscriber = self.create_subscription(Pose, "turtlesim/turtle1/pose", self.pos_or_callback, 10)
        

    def timer_callback(self):

        return
    
    def goal_move_callback(self, msg):
        goal = msg
        t_req = math.sqrt(2*(self.platform_height-self.goal.z)/self.g)
        goal_distance = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        
        self.state = State.BRICK_PUBLISHED
        if goal_distance/self.max_velocity <= t_req:
            self.reachable = True
        else:
            text_reachable = Marker(type=9)
            text_reachable.header.frame_id = "platform_tilt"
            text_reachable.header.stamp = self.get_clock().now().to_msg()
            text_reachable.action = 0
            text_reachable.scale.z = self.wheel_radius*2.5
            text_reachable.id = 10
            text_reachable.pose = Pose(x=0.0,y=0.0,z=self.wheel_radius)
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