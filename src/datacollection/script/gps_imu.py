#!/usr/bin/python3

import rospy
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from data_generator import DataGenerator


class ExtrinsicIntrinsic(DataGenerator):

    def __init__(self) -> None:
        super().__init__()

    
    def camera_vio_callback(self, odo_data, image):
        position = odo_data.pose.pose.position
        self.x = position.x 
        self.y = position.y 
        self.z = position.z 
        # self.w = position.w

        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image 
        self.pub_image = image 
        
        self.signal_recieved = True 

if __name__ == "__main__":
    rospy.init_node("data_generator")

    data_generator = ExtrinsicIntrinsic()

    sub_hires = message_filters.Subscriber(
        "/hires", 
        Image,
    )
    
    sub_odo = message_filters.Subscriber(
        "/qvio/odometry",
        Odometry,   
    )

    subscriber_list = [sub_odo, sub_hires]
    timeSync = message_filters.ApproximateTimeSynchronizer(subscriber_list, queue_size=10, slop=0.1, allow_headerless=True)
    timeSync.registerCallback(data_generator.camera_vio_callback)
    
    data_generator.save_data()