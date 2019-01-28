#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: 
Version:
"""

import rospy

from sensor_msgs.msg import Image
from kobuki_msgs.msg import Sound
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
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
	self.sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
        
	self.prev_scan = []
	self.average = 0.0
	self.alpha = 0
	self.beep = Sound()
	rospy.spin()

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """
        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
	shape = depth[:, depth.shape[1] / 2]
	
	
	if len(self.prev_scan) == 0:
		self.prev_scan = shape

	diff = self.prev_scan - shape
	diff = np.abs(diff)
	diff = diff[~np.isnan(diff)]
	d = np.sum(diff)

	self.alpha = 0.85
	self.average = (self.average * self.alpha) + d * (1 - self.alpha)
	rospy.loginfo("AVERAGE = %f", self.average)
	ratio = d / self.average
	rospy.loginfo("ratio = %f", ratio)
	if (ratio > 1.55):
		self.beep.value = 0
		self.sound.publish(self.beep)

	# After you do things
	self.prev_scan = shape
	
if __name__ == "__main__":
    SentryNode()
