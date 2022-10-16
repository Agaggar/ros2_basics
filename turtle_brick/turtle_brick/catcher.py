from time import sleep
import rclpy, math
from rclpy.node import Node
import rclpy.time
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from enum import Enum, auto
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class State(Enum):
    CHILLING = auto()
    BRICK_FALLING = auto()

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
        self.goal_pub = self.create_publisher(Point, "goal_message",1)
        # assert all values greater than 0

        self.prev_brick_z1 = None
        self.prev_brick_z2 = None
        self.current = Point(x=0.0,y=0.0,z=self.platform_height)
        self.current_pos = Pose(x=0.0,y=0.0,theta=0.0,linear_velocity=0.0,angular_velocity=0.0)
        self.pos_or_subscriber = self.create_subscription(Pose, "turtle1/pose", self.pos_or_callback, 10)
        self.reachable_pub = self.create_publisher(Marker, "/text_marker", 10)

        self.text_reachable = Marker(type=9)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)    

    def timer_callback(self):
        try:
            self.world_brick = self.tf_buffer.lookup_transform(
                "world", "brick", rclpy.time.Time())
        except:
            # print("not published yet")
            return
        if self.world_brick:
            print(self.world_brick.transform.translation.z)
            if self.prev_brick_z1 == None:
                self.prev_brick_z1 = self.world_brick.transform.translation.z
            else:
                if (self.prev_brick_z1 - self.world_brick.transform.translation.z) > 0.0:
                    if self.prev_brick_z2 == None:
                        self.prev_brick_z2 = self.world_brick.transform.translation.z
                    else:
                        if (self.prev_brick_z2 - self.world_brick.transform.translation.z) > 0.0:
                            self.state = State.BRICK_FALLING
                        else:
                            self.prev_brick_z1 = self.world_brick.transform.translation.z
                            self.prev_brick_z2 = None
                else:
                    self.prev_brick_z1 = self.world_brick.transform.translation.z
        # print(self.prev_brick_z1, self.prev_brick_z2, self.world_brick.transform.translation.z)
        if self.state == State.BRICK_FALLING:
            self.check_goal()
        return
    
    def check_goal(self):
        goal = self.world_brick.transform.translation
        height_goal = goal.z - self.platform_height
        if height_goal >= 0:
            t_req = math.sqrt(2*(goal.z-self.platform_height)/self.g)
        else:
            t_req = 0.0
        distance_goal = math.sqrt((goal.y-self.current_pos.y)**2+(goal.x-self.current_pos.x)**2)
        print(goal.x, goal.y, self.current_pos.x, self.current_pos.y, height_goal, distance_goal, t_req)
        if distance_goal/self.max_velocity <= t_req and height_goal >= .5*self.g*t_req**2:
            self.reachable = True
            self.goal_pub.publish(Point(x=goal.x,y=goal.y,z=goal.z))
        else:    
            self.text_reachable.header.frame_id = "platform_tilt"
            self.text_reachable.header.stamp = self.get_clock().now().to_msg()
            self.text_reachable.action = 0
            self.text_reachable.scale.z = self.wheel_radius*2.5
            self.text_reachable.id = 10
            self.text_reachable.pose.position.z = self.wheel_radius
            self.text_reachable.text = "Unreachable"
            self.text_reachable.lifetime = Duration(sec=3,nanosec=0)
            self.text_reachable.color.a = 1.0
            self.reachable_pub.publish(self.text_reachable)
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