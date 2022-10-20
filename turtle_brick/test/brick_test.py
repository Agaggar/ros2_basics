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
    """Generate test description."""
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

    def test_cmd_vel_pub_rate(self, launch_service, cmd_vel_action, proc_output):
        """This function should be checking rate of publishing."""
        # proc_output.assertWaitFor("Publishing turtle1/cmd_vel at:
        #   ", process=cmd_vel_action, timeout=3.0)
        # rclpy.spin_once(self.node)
        # wait_for_topics = WaitForTopics([('turtle1/cmd_vel', Twist)])
        # assert wait_for_topics.wait()
        # print('Given topics are receiving messages !')
        # print(wait_for_topics.topics_not_received()) # Should be an empty set
        # print(wait_for_topics.topics_received())
        # wait_for_topics.shutdown()

# import unittest
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import launch_testing
# import pytest
# import rclpy
# import rclpy.time
# import time
# import os
# # from launch_testing_ros import WaitForTopics
# from geometry_msgs.msg import Twist


# @pytest.mark.rostest
# def generate_test_description():
#     """Generate test description."""
#     path_to_test = os.path.dirname(__file__)
#     turtle_robot_action = Node(package="turtle_brick",
#                                executable="turtle_robot",
#                                name='turtle_robot',
#                                arguments=[os.path.join(path_to_test, 'turtle_robot.py')],
#                                additional_env={'PYTHONUNBUFFERED': '1'},
#                                remappings=[('turtle1/cmd_vel', 'cmd_vel')]
#                                )
#     return (
#         LaunchDescription([
#             turtle_robot_action,
#             launch_testing.actions.ReadyToTest()
#             ]), {'turtle_robot': turtle_robot_action}
#             )


# class TestCmdVelPub(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         """Runs one time, when the testcase is loaded."""
#         rclpy.init()

#     @classmethod
#     def tearDownClass(cls):
#         """Runs one time, when testcase is unloaded."""
#         rclpy.shutdown()

#     def setUp(self):
#         """Runs before every test - create node."""
#         self.node = rclpy.create_node('test_node')

#     def tearDown(self):
#         """Runs after every test - destroy node."""
#         self.node.destroy_node()

#     def test_cmd_vel_pub_rate(self, launch_service, turtle_robot, proc_output):
#         """This function should be checking rate of publishing."""
#         msgs_recieved = []

#         sub = self.node.create_subscription(
#             Twist,
#             'cmd_vel',
#             lambda msg: msgs_recieved.append(msg),
#             10
#         )
#         try:
#             # Wait until the talker transmits two messages over the ROS topic
#             end_time = time.time() + rclpy.time.Duration(seconds=1.0)
#             while time.time() < end_time:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)
#                 if len(msgs_recieved) > 9:
#                     break

#             self.assertGreater(len(msgs_recieved), 9)
#             for msg in msgs_recieved:
#                 proc_output.assertWaitFor(
#                     expected_output=msg.data, process=turtle_robot
#                 )

#         finally:
#             self.node.destroy_subscription(sub)

#         # proc_output.assertWaitFor("Publishing turtle1/cmd_vel at:
#         #   ", process=cmd_vel_action, timeout=3.0)
#         # rclpy.spin_once(self.node)
#         # wait_for_topics = WaitForTopics([('turtle1/cmd_vel', Twist)])
#         # assert wait_for_topics.wait()
#         # print('Given topics are receiving messages !')
#         # print(wait_for_topics.topics_not_received()) # Should be an empty set
#         # print(wait_for_topics.topics_received())
#         # wait_for_topics.shutdown()
