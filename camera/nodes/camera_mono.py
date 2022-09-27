#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


class CameraMono(object):
    def __init__(self):
        self.line_idx_pub = rospy.Publisher("line_idx", UInt32, queue_size=1)
        self.camera_subscriber = rospy.Subscriber(
            "camera/image", Image, self.camera_callback, queue_size=1
        )
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)

        # average each column (removing some rows): result is a 640 length array
        x = np.mean(cv_img[100:300, :], axis=0)

        # efficient moving window filter
        # see <https://stackoverflow.com/a/27681394>
        r = 5
        w = 2 * r + 1  # total filter width
        c = np.cumsum(np.insert(x, 0, 0))
        a = (c[w:] - c[:-w]) / w

        # add r because we removed r elements on each side of the array doing the averaging
        idx = np.argmin(a) + r

        self.line_idx_pub.publish(idx)


def main():
    rospy.init_node("camera_mono")
    camera = CameraMono()
    rospy.spin()


if __name__ == "__main__":
    main()
