import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
import rclpy.time
import time
import os
from geometry_msgs.msg import Twist


@pytest.mark.rostest
def generate_test_description():
    """Generate test description."""
    path_to_test = os.path.dirname(__file__)
    turtle_robot_action = Node(package="turtle_brick",
                               executable="turtle_robot",
                               arguments=[os.path.join(path_to_test, 'turtle_robot.py')],
                               additional_env={'PYTHONUNBUFFERED': '1'}
                               )
    return (
        LaunchDescription([
            turtle_robot_action,
            launch_testing.actions.ReadyToTest()
            ]), {'turtle_robot': turtle_robot_action}
            )


class TestCmdVelPub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Run one time, when the testcase is loaded."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Run one time, when testcase is unloaded."""
        rclpy.shutdown()

    def setUp(self):
        """Run before every test - create node."""
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Run after every test - destroy node."""
        self.node.destroy_node()

    def test_cmd_vel_pub_rate(self, launch_service, turtle_robot, proc_output):
        """
        This function checks the rate of publishing.

        Spin the node once, which appends a message, if subscribed, as well as the time.
        Append the message to a list, and spin again.
        Assert that the length is > 9, and the time stamps differ by ~0.01 s.
        """
        msgs_recieved = []
        time_recieved = []

        sub = self.node.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            lambda msg: msgs_recieved.append(msg),
            10
        )
        try:
            # Wait until the talker transmits two messages over the ROS topic
            end_time = time.time() + 3.0
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                # print(msgs_recieved)
                time_recieved.append(time.time())
                if len(msgs_recieved) > 10:
                    break

            difference = []
            self.assertGreater(len(msgs_recieved), 10)
            for index in range(len(time_recieved)):
                if index == 0:
                    pass
                else:
                    difference.append(time_recieved[index] - time_recieved[index - 1])
            avg = sum(difference)/len(difference)
            self.assertGreaterEqual(abs(avg - 0.001), 0.0)

        finally:
            self.node.destroy_subscription(sub)
