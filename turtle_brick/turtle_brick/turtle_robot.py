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

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

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
        self.pos_or_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pos_or_callback, 10)
        self.current = None
        self.spawn_pos = None
        self.odom_bool = False

        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if self.odom_bool == True:
            world_base_tf = TransformStamped()
            world_base_tf.header.stamp = self.get_clock().now().to_msg()
            world_base_tf.header.frame_id = "world"
            world_base_tf.child_frame_id = "odom"
            world_base_tf.transform.translation.x = self.spawn_pos.x
            world_base_tf.transform.translation.y = self.spawn_pos.y
            self.static_broadcaster.sendTransform(world_base_tf)

        odom_base_link = TransformStamped()
        odom_base_link.header.stamp = self.get_clock().now().to_msg()
        odom_base_link.header.frame_id = "odom"
        odom_base_link.child_frame_id = "base_link"
        odom_base_link.transform.translation.z = 4.0
        # odom_base_link.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])
        self.broadcaster.sendTransform(odom_base_link)
    
    def pos_or_callback(self, msg):
        """Called by self.pos_or_subscriber
        Subscribes to pose, and updates current with pose
        """
        if (self.spawn_pos == None):
            self.spawn_pos = msg
            self.odom_bool = True
        self.current = msg
        self.get_logger().debug(f'I heard: {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()