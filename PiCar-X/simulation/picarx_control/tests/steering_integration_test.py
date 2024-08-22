#!/usr/bin/env python3

import time
import unittest

import rospy
import rosunit

from picarx_msgs.msg import Drive
from gazebo_msgs.msg import LinkStates

class Test_Speed(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("SPEEDTEST", anonymous=True)
        self.inital_message = None
        self.final_message = None
        self.finished = False
        self.send_test_publisher = rospy.Publisher(
            '/picarx/drive/command', Drive, queue_size=5)
        if self.send_test_publisher.get_num_connections() == 0:
            time.sleep(0.3)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.subscribe_initial_message)

    def subscribe_initial_message(self, msg: LinkStates):
        if self.inital_message is None:
            self.inital_message = msg
        else:
            if self.finished == True:
                if self.final_message is None:
                    self.final_message = msg

    def test_dt_pub_data(self):
        start_message = Drive(rospy.Time.now(), 100, 0)
        stop_message = Drive(rospy.Time.now(), 0, 0)
        time.sleep(0.3)

        self.send_test_publisher.publish(start_message)
        time.sleep(1.5)
        self.send_test_publisher.publish(stop_message)
        self.finished = True
        time.sleep(0.1)
        right_wheel_start_index = self.inital_message.name.index('picarx::right_wheel')
        left_wheel_start_index = self.inital_message.name.index('picarx::left_wheel')
        right_wheel_stop_index = self.final_message.name.index('picarx::right_wheel')
        left_wheel_stop_index = self.final_message.name.index('picarx::left_wheel')


        right_wheel_start_position = self.inital_message.pose[right_wheel_start_index]
        left_wheel_start_position = self.inital_message.pose[left_wheel_start_index]
        right_wheel_stop_position = self.final_message.pose[right_wheel_stop_index]
        left_wheel_stop_position = self.final_message.pose[left_wheel_stop_index]
        rospy.loginfo(right_wheel_start_position)
        self.assertAlmostEqual(right_wheel_start_position.position.x + 1, right_wheel_stop_position.position.x, delta=0.04)
        self.assertAlmostEqual(left_wheel_start_position.position.x + 1, left_wheel_stop_position.position.x, delta=0.04)


if __name__ == '__main__':
    rosunit.unitrun("ackermann_skill", 'test_pt_speed',
                    Test_Speed)