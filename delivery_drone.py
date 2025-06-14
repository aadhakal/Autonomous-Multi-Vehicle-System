# -*- coding: utf-8 -*-
#!/usr/bin/python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the vehicle
vehicle = connect('udp:127.0.0.1:14560', wait_ready=True, timeout=60)

# ROS setup
rospy.init_node('delivery_drone_node', anonymous=False)

# Publishers and Subscribers
ack_pub = rospy.Publisher('/location_ack', String, queue_size=10)
processed_image_pub = rospy.Publisher('drone2/camera/color/image_new', Image, queue_size=10)

# Global Variables
marker_lat = None
marker_lon = None
signal_received = False
time_last = 0
time_to_wait = 0.1
takeoff_height = 3
# ArUco Detection Parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
camera_matrix = [[530.8269276712998, 0.0, 320.5],
                 [0.0, 530.8269276712998, 240.5],
                 [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
distortion_coefficients = [0.0, 0.0, 0.0, 0.0, 0.0]
np_distortion_coefficients = np.array(distortion_coefficients)

horizontal_res = 640  # in pixels
vertical_res = 480  # in pixels
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * 2 * (math.pi / 180)

# Callback for receiving marker location from escort drone
def marker_location_callback(data):
    global marker_lat, marker_lon, signal_received
    try:
        lat, lon = map(float, data.data.split(','))
        if -90 <= lat <= 90 and -180 <= lon <= 180:
            marker_lat = lat
            marker_lon = lon
            signal_received = True
            print("Received marker location: Latitude=" + str(marker_lat) + " Longitude=" + str(marker_lon) + " ")

            # Send acknowledgment back to escort drone
            ack_pub.publish("ACK_RECEIVED")
            print("Acknowledgment sent to escort drone.")
        else:
            print("Invalid GPS coordinates received.")
    except ValueError as e:
        print("Error parsing location data: " + str(e))

rospy.Subscriber('/aruco_marker_location', String, marker_location_callback)

# Process camera images for precision landing
def process_image(message, marker_id):
    global time_last
    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        (corners, ids, rejected) = aruco.detectMarkers(
            gray_image, dictionary=aruco_dict, parameters=parameters
        )

        try:
            if ids is not None:
                for idx, detected_id in enumerate(ids.flatten()):
                    print("Detected marker ID: " + str(detected_id) + " ")

                    if detected_id == marker_id:
                        print("Marker ID " + str(marker_id) + " found. Adjusting position for precision landing.")
                        corners_single = [corners[idx]]
                        ret = aruco.estimatePoseSingleMarkers(
                            corners_single, 20,
                            cameraMatrix=np_camera_matrix,
                            distCoeffs=np_distortion_coefficients
                        )
                        _, tvec = ret[0][0][0], ret[1][0][0]
                        x_ang = (tvec[0] - horizontal_res * 0.5) * (horizontal_fov / horizontal_res)
                        y_ang = (tvec[1] - vertical_res * 0.5) * (vertical_fov / vertical_res)
                        send_landing_message(x_ang, y_ang)
                        return

            # Publish the processed image
            new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
            processed_image_pub.publish(new_msg)

        except Exception as e:
            print("Error processing markers: " + str(e) + " ")

        time_last = time.time()

# Publish landing message for precision alignment
def send_landing_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, x, y, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Navigate to a GPS location
def goto(target_location):
    print("Flying to target location: lat=" + str(target_location.lat) + " lon=" + str(target_location.lon) + " ")
    vehicle.simple_goto(target_location)
    while vehicle.mode.name == "GUIDED":
        current_distance = get_distance_meters(target_location, vehicle.location.global_relative_frame)
        print("Distance to target: " + str(current_distance) + " meters ")
        if current_distance < 1:
            print("Reached target location. ")
            break
        time.sleep(1)

# Calculate distance in meters between two GPS locations
def get_distance_meters(a_location, b_location):
    dlat = a_location.lat - b_location.lat
    dlon = a_location.lon - b_location.lon
    return math.sqrt((dlat ** 2) + (dlon ** 2)) * 1.113195e5

# Arm and Takeoff
def arm_and_takeoff(target_height):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable... ")
        time.sleep(1)

    print("Vehicle is armable. ")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for GUIDED mode... ")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to arm... ")
        time.sleep(1)

    vehicle.simple_takeoff(target_height)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Current Altitude: " + str(current_altitude) + " ")
        if current_altitude >= 0.95 * target_height:
            print("Target altitude reached. ")
            break
        time.sleep(1)

if __name__ == "__main__":
    try:
        # Save home location
        home_lat = vehicle.location.global_relative_frame.lat
        home_lon = vehicle.location.global_relative_frame.lon
        print("Home location saved: lat=" + str(home_lat) + " lon=" + str(home_lon) + " ")

        # Monitor for marker location signals
        while True:
            if signal_received:
                # Fly to marker location
                target_location = LocationGlobalRelative(marker_lat, marker_lon, takeoff_height)
                arm_and_takeoff(takeoff_height)
                goto(target_location)

                # Perform precision landing using marker ID 72
                print("Performing precision landing at marker ID 72.")
                rospy.Subscriber('drone2/camera/camera/color/image_raw', Image, process_image, callback_args=72)
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    time.sleep(1)

                # Return to home and perform precision landing using marker ID 18
                print("Returning to home location... ")
                arm_and_takeoff(takeoff_height)
                goto(LocationGlobalRelative(home_lat, home_lon, takeoff_height))
                print("Performing precision landing at home marker ID 18.")
                rospy.Subscriber('drone2/camera/camera/color/image_raw', Image, process_image, callback_args=18)
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    time.sleep(1)

                # Reset signal for next task
                signal_received = False
                marker_lat = None
                marker_lon = None
            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
    finally:
        print("Mission complete. Closing vehicle connection.")
        vehicle.close()
