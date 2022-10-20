import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
# from launch_testing_ros import WaitForTopics
# from geometry_msgs.msg import Twist


@pytest.mark.rostest
def generate_test_description():
    """Generate test description"""
    turtle_robot_action = Node(package="turtle_brick",
                               executable="turtle_robot",
                               name='turtle_robot')
    return (
        LaunchDescription([
            turtle_robot_action,
            launch_testing.actions.ReadyToTest()
            ]), {'cmd_vel_action': turtle_robot_action}
            )


class TestCmdVelPub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs one time, when the testcase is loaded"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Runs one time, when testcase is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test - create node"""
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Runs after every test - destroy node"""
        self.node.destroy_node()

    def test_cmd_vel_pub_rate(self, launch_service, cmd_vel_action, proc_output):
        """This function should be checking rate of publishing"""
        # proc_output.assertWaitFor("Publishing turtle1/cmd_vel at: 
        #   ", process=cmd_vel_action, timeout=3.0)
        # rclpy.spin_once(self.node)
        # wait_for_topics = WaitForTopics([('turtle1/cmd_vel', Twist)])
        # assert wait_for_topics.wait()
        # print('Given topics are receiving messages !')
        # print(wait_for_topics.topics_not_received()) # Should be an empty set
        # print(wait_for_topics.topics_received())
        # wait_for_topics.shutdown()
