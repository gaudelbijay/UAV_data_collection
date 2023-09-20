#!/usr/bin/python3
import os 
import cv2 
import rospy
import numpy as np 
import pandas as pd
from datetime import datetime


class DataGenerator(object):
    def __init__(self,) -> None:
        super().__init__()

        self.x = 0 
        self.y = 0
        self.z = 0
        self.pub_image = np.zeros((256,256))

        self.time = rospy.get_param("~time")
        self.weather = rospy.get_param("~weather")
        self.site_name = rospy.get_param("~site_name")
        self.camera_name = rospy.get_param("~camera_name")
        self.parent_directory = rospy.get_param("~parent_directory")

        self.signal_recieved = True  

        self.df = pd.DataFrame()
        self.rate = rospy.Rate(1)
        self.file_name = "info.csv"
        self.img_prefix = 0

        self.current_directory = os.getcwd()
        self.site_folder = f"{self.current_directory}/{self.parent_directory}/{self.site_name}"
        self.camera_folder = f"{self.site_folder}/{self.camera_name}"

        self.root_directory()
        self.create_next_integer_folders()
    
    def root_directory(self,):
        os.makedirs(self.parent_directory, exist_ok=True)
        os.makedirs(self.site_folder, exist_ok=True)
        os.makedirs(self.camera_folder, exist_ok=True)

    def get_last_integer_folder(self,):
        all_folders = [d for d in os.listdir(self.camera_folder) if os.path.isdir(os.path.join(self.camera_folder, d))]
        integer_folders = [int(folder) for folder in all_folders if folder.isdigit()]

        if not integer_folders:
            return 0
        return max(integer_folders)
        
    def create_next_integer_folders(self,):
        last_integer_folder = self.get_last_integer_folder()
        new_folder_name = str(last_integer_folder + 1)
        new_folder_path = os.path.join(self.camera_folder, new_folder_name)
        os.makedirs(new_folder_path, exist_ok=True)

    def save_data(self,):
        folder = str(self.get_last_integer_folder())
        while not rospy.is_shutdown():
            if not self.signal_recieved:
                self.rate.sleep()

            current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            self.pub_image_to_save = f"{self.camera_folder}/{folder}/image_"+str(self.img_prefix)+".jpg"

            cv2.imwrite(self.pub_image_to_save, self.pub_image)

            self.df = self.df.append({
                "x": self.x, "y": self.y, "z": self.z,
                "time_stamp":current_timestamp,
                "image": self.pub_image_to_save,
            }, ignore_index=True)

            self.img_prefix += 1
            self.df.to_csv(f"{self.camera_folder}/{folder}/{self.file_name}", index=False)
            self.rate.sleep()