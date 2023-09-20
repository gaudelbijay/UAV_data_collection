#!/usr/bin/python3
import rospy
import message_filters
# import tf
from std_msgs.msg import Float64
from datetime import datetime
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs

from data_generator import DataGenerator



class ExtrinsicIntrinsic(DataGenerator):

    def __init__(self) -> None:
        super(ExtrinsicIntrinsic, self).__init__()
        self.signal_recieved = False 

        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

        self.quaternion = (
           0,0,0,0,
        )

        self.quaternion_position_imu = (
           0,0,0,0,

        )

        self.triplet_angular_velocity_imu = (
           0,0,0,

        )

        self.triplet_linear_acceleration_imu = (
            0,0,0,

        )

        self.compass_hdg = 0

    
    def camera_gps_imu_callback(self, imu_data):



        self.quaternion_position_imu = (
            imu_data.orientation.x, 
            imu_data.orientation.y, 
            imu_data.orientation.z,
            imu_data.orientation.w,

        )

        self.triplet_angular_velocity_imu = (
            imu_data.angular_velocity.x, 
            imu_data.angular_velocity.y, 
            imu_data.angular_velocity.z,

        )

        self.triplet_linear_acceleration_imu = (
            imu_data.linear_acceleration.x, 
            imu_data.linear_acceleration.y, 
            imu_data.linear_acceleration.z,

        )

        self.signal_recieved = True

    def save_data(self,):
        folder = str(self.get_last_integer_folder())
        current_time = datetime.now()
        self.day = current_time.day
        self.month = current_time.month
        self.year = current_time.year
        self.hour = current_time.hour
        while not rospy.is_shutdown():
            # if not self.signal_recieved:
            #     self.rate.sleep()
                
            self.df = self.df.append({
                "time": current_time, 
                "lat": self.latitude, 
                "long": self.longitude,
                "alt": self.altitude,
                # "roll": self.roll,
                # "pitch": self.pitch,
                # "yaw": self.yaw,
                "gps_x": self.quaternion[0],
                "gps_y": self.quaternion[1],
                "gps_z": self.quaternion[2],
                "gps_w": self.quaternion[3], 
                "imu_x": self.quaternion_position_imu[0],
                "imu_y": self.quaternion_position_imu[1],
                "imu_z": self.quaternion_position_imu[2],
                "imu_w": self.quaternion_position_imu[3],
                "angualr_velocity_imu_x": self.triplet_angular_velocity_imu[0],
                "angualr_velocity_imu_y": self.triplet_angular_velocity_imu[1],
                "angualr_velocity_imu_z": self.triplet_angular_velocity_imu[2],
                "linear_acceleration_imu_x": self.triplet_linear_acceleration_imu[0],
                "linear_acceleration_imu_y": self.triplet_linear_acceleration_imu[1],
                "linear_acceleration_imu_z": self.triplet_linear_acceleration_imu[2],
                "heading": self.compass_hdg,
                "time": self.hour,
                "day": self.day,
                "month": self.month,
                "year": self.year,
                "camera": "GOPRO11-MINI-BLACK",
                
            }, ignore_index=True)

            self.img_prefix += 1
            self.df.to_csv(f"{self.camera_folder}/{folder}/{self.file_name}", index=False)
            self.rate.sleep()
    
        
if __name__ == "__main__":
    rospy.init_node("data_generator")

    data_generator = ExtrinsicIntrinsic()

    sub_gps_global = message_filters.Subscriber(
        "/mavros/global_position/global",
        sensor_msgs.NavSatFix,
    )

    sub_gps_local = message_filters.Subscriber(
        "/mavros/global_position/local",
        geometry_msgs.PoseStamped,
        
    )

    sub_compass_hdg = message_filters.Subscriber(
        "/mavros/global_position/compass_hdg",
        Float64
    )

    sub_imu = message_filters.Subscriber(
        "/mavros/imu/data",
        sensor_msgs.Imu
    )

    # subscriber_list = [sub_gps_global, sub_gps_local, sub_imu, sub_compass_hdg]
    subscriber_list = [sub_imu]
    timeSync = message_filters.ApproximateTimeSynchronizer(subscriber_list, queue_size=10, slop=0.1, allow_headerless=True)
    timeSync.registerCallback(data_generator.camera_gps_imu_callback)
    
    data_generator.save_data()