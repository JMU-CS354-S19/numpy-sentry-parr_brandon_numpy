#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Authors: Brandon Parr and Nathan Moore
Version: 1.0
"""

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        self.p = None
        self.average = 1.0
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        rospy.spin()

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """

        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.
        middle = depth.shape[1] / 2
        c = depth[:,middle]
        c = c[~np.isnan(c)]

        d = 0

        if (self.p != None):
        	d = np.sum(np.abs(c - self.p))

        alpha = 0.5
        self.average = (self.average * alpha) + (d * (1 - alpha))

        temp = d / self.average
        rospy.loginfo(temp)


        self.p = c






if __name__ == "__main__":
    SentryNode()
