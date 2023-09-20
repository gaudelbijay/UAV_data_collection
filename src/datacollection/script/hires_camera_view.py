#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/hires", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Could not convert image: %s" % e)
            return

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)  # Display the image for a short duration. Adjust as needed.

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    img_sub = ImageSubscriber()
    rospy.spin()  # Keep the node running until manually terminated.

    cv2.destroyAllWindows()  # Close any OpenCV windows.
