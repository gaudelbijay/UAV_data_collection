from pymavlink import mavutil
import csv
import datetime
import time
import numpy as np


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


def data_stream(udp_url='udp:172.17.0.1:14550', file_name='info.csv', tilt_angle_from_x = 0):
    # Connect to the drone's MAVLink instance.
    mavlink_connection = mavutil.mavlink_connection(udp_url)

    # Define the CSV file and its columns
    filename = file_name
    fieldnames = ['datetime', 'latitude', 'longitude', 'altitude', 'timestamp', 'odo_x', 'odo_y', 'odo_z', 'pressure_alt', 
                'temperature', 'camera_roll', 'camera_pitch', 'camera_yaw', 'rollspeed', 'pitchspeed', 'yawspeed', 'heading']

    # Open the CSV file for writing
    with open(filename, 'w', newline='') as csv_file:
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
                    'altitude': msg.alt / 1e3
                })
            elif msg.get_type() == 'HIGHRES_IMU':
                data.update({
                    'timestamp': msg.time_usec,
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

            elif msg.get_type() == 'ODOMETRY':
                data.update({
                    'odo_x': msg.x,
                    'odo_y': msg.y,
                    'odo_z': msg.z,
                })

            elif msg.get_type() == 'VFR_HUD':
                data.update({
                    'heading': (msg.heading + 180) % 360,
                })

            # Check if one second has passed
            if time.time() - start_time >= 1.0:
                data['datetime'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                writer.writerow(data)
                data = {}  # Reset data for next iteration
                start_time = time.time()  # Reset timer


if __name__ == "__main__":
    data_stream()