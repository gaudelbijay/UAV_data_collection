#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher:
    def __init__(self, rtsp_url):
        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open the RTSP stream.")
            raise RuntimeError("Failed to open the RTSP stream.")

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=10)

    def publish_frame(self):
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if ret:
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.image_pub.publish(ros_image)
                    except Exception as e:
                        rospy.logerr("Error: %s" % e)
                rate.sleep()
        finally:
            self.cap.release()

if __name__ == '__main__':
    rospy.init_node('camera_publisher', anonymous=True)
    rtsp_url = rospy.get_param('~rtsp_url', "rtsp://172.16.0.70:8900/live")  # default URL if param not set
    camera_pub = CameraPublisher(rtsp_url)
    camera_pub.publish_frame()
