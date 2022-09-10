#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32


class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

    def camera_callback(self, msg):
        """Callback for line index."""
        # access the value using msg.data
        pass

    def follow_the_line(self):
        """
        TODO: complete the function to follow the line
        """
        pass


if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line()
