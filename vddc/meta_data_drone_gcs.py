from pymavlink import mavutil
import csv
import datetime
import time
import numpy as np
import os 
import threading
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


# Global lists for latitude and longitude
latitudes = []
longitudes = []

class DataGenerator(object):
    def __init__(self, parent_directory, camera_name, site_name) -> None:
        super().__init__()

        self.site_name = site_name
        self.camera_name = camera_name
        self.parent_directory = parent_directory

        self.signal_recieved = True  

        self.file_name = "info.csv"

        self.current_directory = os.getcwd()
        self.site_folder = f"{self.current_directory}/{self.parent_directory}/{self.site_name}"
        self.camera_folder = f"{self.site_folder}/{self.camera_name}"

        self.root_directory()
        # self.create_next_integer_folders()
    
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
        return new_folder_path

def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta[0]), -np.sin(theta[0])],
                    [0, np.sin(theta[0]), np.cos(theta[0])]
                    ])
    R_y = np.array([[np.cos(theta[1]), 0, np.sin(theta[1])],
                    [0, 1, 0],
                    [-np.sin(theta[1]), 0, np.cos(theta[1])]
                    ])
    R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def update_plot(frame):
    lat, lon = frame
    latitudes.append(lat)
    longitudes.append(lon)
    ax.clear()
    ax.plot(longitudes, latitudes, color='blue', marker='o', markersize=4)
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('GPS Data Stream')

# udp:0.0.0.0:14551 for drone 
def data_stream(udp_url='udp:172.16.0.164:14550', file_name='info.csv', tilt_angle_from_x = 50):
    # Connect to the drone's MAVLink instance.
    mavlink_connection = mavutil.mavlink_connection(udp_url)

    # Define the CSV file and its columns
    filename = file_name
    fieldnames = fieldnames = [
        'datetime', 'latitude', 'longitude', 'altitude', 
        'eph_gps', 'epv_gps', 'vel_gps', 'satellites_visible',
        'alt_elipsoid', 'h_acc_gps', 'v_acc_gps', 'vel_acc_gps', 'hdg_acc_gps',
        'xacc_imu', 'yacc_imu', 'zacc_imu', 'xgyro_imu',
        'ygyro_imu', 'zgyro_imu', 'xmag_imu', 'ymag_imu',
        'zmag_imu', 'pressure_alt', 'temperature',
        'camera_roll', 'camera_pitch', 'camera_yaw',
        'rollspeed', 'pitchspeed', 'yawspeed',
        'q1', 'q2', 'q3', 'q4', 'repr_offset_q',
        'odo_x', 'odo_y', 'odo_z', 'odo_vx', 'odo_vy', 'odo_q', 'odo_velocity_covariance', 'odo_pose_covariance',
        'odo_vz', 'odo_rollspeed', 'odo_pitchspeed', 'odo_yawspeed',
        'groundspeed', 'throttle', 'climb', 'heading'
    ]

    dg = DataGenerator("metadata", "GOPROMINI11", "StevensLibrary")
    folder = dg.create_next_integer_folders()
    csv_file_path = os.path.join(folder, file_name)

    # Open the CSV file for writing
    with open(csv_file_path, 'w', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()  # Write the column headers
        
        data = {}
        start_time = time.time()
        
        while True:
            # Receive a message
            msg = mavlink_connection.recv_match(blocking=True)

            # Update data dictionary based on message type
            if msg.get_type() == 'GPS_RAW_INT':
                data.update({
                    'latitude': msg.lat / 1e7,
                    'longitude': msg.lon / 1e7,
                    'altitude': msg.alt / 1e3,
                    'eph_gps':msg.eph, 
                    'epv_gps': msg.epv, 
                    'vel_gps': msg.vel,
                    'satellites_visible': msg.satellites_visible, 
                    'alt_elipsoid': msg.alt_ellipsoid, 
                    'h_acc_gps': msg.h_acc,
                    'v_acc_gps': msg.v_acc,
                    'vel_acc_gps': msg.vel_acc, 
                    'hdg_acc_gps': msg.hdg_acc
                })
                # yield msg.lat/1e7, msg.lon/1e7
                
            elif msg.get_type() == 'HIGHRES_IMU':
                data.update({
                    'xacc_imu': msg.xacc,
                    'yacc_imu': msg.yacc, 
                    'zacc_imu': msg.zacc, 
                    'xgyro_imu': msg.xgyro,
                    'ygyro_imu': msg.ygyro, 
                    'zgyro_imu': msg.zgyro,
                    'xmag_imu': msg.xmag,
                    'ymag_imu': msg.ymag, 
                    'zmag_imu': msg.zmag,
                    'pressure_alt': msg.pressure_alt,
                    'temperature': msg.temperature,
                })
            elif msg.get_type() == 'ATTITUDE':
                drone_angles = np.array([msg.roll, msg.pitch, msg.yaw + np.pi])  # Adding pi (180 degrees) for opposing direction
                alpha = np.radians(tilt_angle_from_x)
                R_tilt = eulerAnglesToRotationMatrix(np.array([0, alpha, 0]))
                R_drone = eulerAnglesToRotationMatrix(drone_angles)
                R_combined = np.dot(R_drone, R_tilt)
                camera_angles = rotationMatrixToEulerAngles(R_combined)
                data.update({
                    'camera_roll': camera_angles[0],
                    'camera_pitch': camera_angles[1],
                    'camera_yaw': camera_angles[2],
                    'rollspeed': msg.rollspeed,
                    'pitchspeed': msg.pitchspeed,
                    'yawspeed': msg.yawspeed,

                })
            elif msg.get_type() == 'ATTITUDE_QUATERNION':
                data.update({
                    'q1': msg.q1,
                    'q2': msg.q2, 
                    'q3': msg.q3, 
                    'q4': msg.q4, 
                    'repr_offset_q': msg.repr_offset_q, 
                })

            elif msg.get_type() == 'ODOMETRY':
                data.update({
                    'odo_x': msg.x,
                    'odo_y': msg.y,
                    'odo_z': msg.z,
                    'odo_q': msg.q,
                    'odo_vx': msg.vx, 
                    'odo_vy': msg.vy,
                    'odo_vz': msg.vz, 
                    'odo_rollspeed': msg.rollspeed, 
                    'odo_pitchspeed': msg.pitchspeed, 
                    'odo_yawspeed': msg.yawspeed, 
                    'odo_pose_covariance': msg.pose_covariance, 
                    'odo_velocity_covariance': msg.velocity_covariance, 
                })
                yield msg.x, msg.y

            elif msg.get_type() == 'VFR_HUD':
                data.update({
                    'groundspeed': msg.groundspeed, 
                    'throttle': msg.throttle, 
                    'climb': msg.climb,
                    'heading': (msg.heading + 180) % 360,
                })

            # Check if one second has passed
            if time.time() - start_time >= 1.0:
                data['datetime'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                writer.writerow(data)
                data = {}  # Reset data for next iteration
                start_time = time.time()  # Reset timer


if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(8, 6))
    global ani  # Declare ani as a global variable
    ani = FuncAnimation(fig, update_plot, data_stream('udp:172.16.0.164:14550'), interval=1000)

    plt.show()
