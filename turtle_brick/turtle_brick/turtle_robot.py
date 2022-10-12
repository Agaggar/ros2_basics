from turtle import Turtle, reset
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
        self.dx = 10  # used to control frame movement
        # create the broadcaster
        # self.static_broadcaster.sendTransform(self)
        # self.broadcaster.sendTransform(self)

        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = "world"
        world_base_tf.child_frame_id = "base"
        self.static_broadcaster.sendTransform(world_base_tf)

        base_odom = TransformStamped()
        base_odom.header.stamp = self.get_clock().now().to_msg()
        base_odom.header.frame_id = "base"
        base_odom.child_frame_id = "odom"
        # base_odom.transform.translation.x = -float(self.dx)
        # get a quaternion corresponding to a rotation by theta about an axis
        self.static_broadcaster.sendTransform(base_odom)
        # Create a timer to do the rest of the transforms

        self.tmr = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        # Now create the transform, noted that it must have a parent frame and a timestamp
        # The header contains the timing information and frame id

        base_brick = TransformStamped()
        base_brick.header.stamp = self.get_clock().now().to_msg()
        base_brick.header.frame_id = "base"
        base_brick.child_frame_id = "brick"
        # base_brick.transform.translation.x = float(self.dx)
        # base_brick.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])
        self.broadcaster.sendTransform(base_brick)

        base_base_link = TransformStamped()
        base_base_link.header.stamp = self.get_clock().now().to_msg()
        base_base_link.header.frame_id = "base"
        base_base_link.child_frame_id = "base_link"
        # base_base_link.transform.translation.x = float(self.dx)
        # base_base_link.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])
        self.broadcaster.sendTransform(base_base_link)

        # update the movement
        self.dx -= 1
        if self.dx == 0:
            self.dx = 10


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()