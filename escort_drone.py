# -*- coding: utf-8 -*-
#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the vehicle
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True, timeout=60) # This port will be the serial port in real hardware, for Examplez: ttyAMA0 for pixhawx
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30

newimage_pub = rospy.Publisher('camera1/color/image_new', Image, queue_size=10)
marker_location_pub = rospy.Publisher('/aruco_marker_location', String, queue_size=10)

# Variables
takeoff_height = 5  # meters
ack_received = False
target_found = False
precision_landing_active = False
home_lat, home_lon = None, None

# ArUco Detection Parameters
ids_to_find = [129, 72]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640 # in pixels
vertical_resolution = 480  #in pixels

horizontal_fov = 62.2 * (math.pi / 180) #depends on a pi cam version
vertical_fov = 48.8 * 2 * (math.pi / 180)

time_to_wait = .1 #100ms
time_last = 0

# Camera Intrinsics
camera_matrix = [[530.8269276712998, 0.0, 320.5],
                 [0.0, 530.8269276712998, 240.5],
                 [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
distortion_coefficient = [0.0, 0.0, 0.0, 0.0, 0.0]
np_distortion_coefficient = np.array(distortion_coefficient)

# Acknowledgment callback
def ack_callback(data):
    global ack_received
    if data.data == "ACK_RECEIVED":
        ack_received = True
        print("Acknowledgment received from the delivery drone.")

ack_sub = rospy.Subscriber('/location_ack', String, ack_callback)


def get_distance_meters(a_location, b_location):
    dlat = a_location.lat - b_location.lat
    dlon = a_location.lon - b_location.lon
    return math.sqrt((dlat ** 2) + (dlon ** 2)) * 1.113195e5

# Utility Functions
def goto(target_location):
    print("Flying to target location: lat=" + str(target_location.lat) + ", lon=" + str(target_location.lon))
    vehicle.simple_goto(target_location)
    while vehicle.mode.name == "GUIDED":
        current_distance = get_distance_meters(target_location, vehicle.location.global_relative_frame)
        print("Distance to target: " + str(current_distance) + " meters")
        if current_distance < 1:
            print("Reached target location.")
            break
        time.sleep(1)

# Arm and Takeoff
def arm_and_takeoff(target_height):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    print("Vehicle is armable.")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)
    print("GUIDED mode activated.")

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)
    print("Vehicle armed.")

    vehicle.simple_takeoff(target_height)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Current Altitude: " + str(current_altitude))
        if current_altitude >= target_height * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)

    return None

# COmputer Vision & Image Processing
def process_image(message):
    global time_last, time_to_wait, ack_received

    # Time-based processing to avoid overloading
    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)  # Deserialize image data into NumPy array
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the frame
        (corners, ids, rejected) = aruco.detectMarkers(
            image=gray_image, dictionary=aruco_dict, parameters=parameters
        )

        marker_size = 40

        try:
            # Process detected IDs
            if ids is not None:
                for idx in range(len(ids)):
                    marker_id = ids[idx][0]
                    print("Detected marker ID: " + str(marker_id))

                    if marker_id == 72:  # Delivery marker
                        send_marker_location(marker_id)

                    elif marker_id == 129:  # Home marker
                        if ack_received:
                        # Process marker position
                            corners_single = [corners[idx]]
                            corners_single_np = np.asarray(corners_single)
                            ret = aruco.estimatePoseSingleMarkers(
                                corners_single, marker_size,
                                cameraMatrix=np_camera_matrix,
                                distCoeffs=np_distortion_coefficient
                            )
                            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                            x = '{:.2f}'.format(tvec[0])  # Distance to marker in cm
                            y = '{:.2f}'.format(tvec[1])
                            z = '{:.2f}'.format(tvec[2])

                            x_sum = 0
                            y_sum = 0
                            x_avg = 0
                            y_avg = 0

                            x_sum = corners_single_np[0][0][0][0] + corners_single_np[0][0][1][0] + corners_single_np[0][0][2][0] + corners_single_np[0][0][3][0] # pixel value avg from corners array
                            y_sum = corners_single_np[0][0][0][1] + corners_single_np[0][0][1][1] + corners_single_np[0][0][2][1] + corners_single_np[0][0][3][1]

                            x_avg = x_sum / 4
                            y_avg = y_sum / 4

                            x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res) # gives the center position of the Aruco
                            y_ang = (y_avg - vertical_resolution * .5) * (vertical_fov / vertical_resolution)

                            if vehicle.mode != 'LAND':
                                vehicle.mode = VehicleMode('LAND')
                                while vehicle.mode != 'LAND':
                                    time.sleep(1)
                                print('Vehicle is in LAND mode')

                            # Always send landing message to adjust position
                            send_landing_message(x_ang, y_ang)

                            marker_position = "MARKER POSITION: x = " + x + " y = " + y + " z = " + z
                            aruco.drawDetectedMarkers(np_data, corners)
                            aruco.drawAxis(np_data, np_camera_matrix, np_distortion_coefficient, rvec, tvec, 10)
                            cv2.putText(np_data, marker_position, (10,50),0,.7, (255,0,0), thickness = 2)
                            print(marker_position)
        except Exception as e:
            print("Error processing markers: " + str(e))

        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimage_pub.publish(new_msg)
    else:
        return None

def send_landing_message(x, y):  # distance between the drone and the target
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def lawnmower_motion(lateral_distance, forward_increment, max_distance):
    global sub
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('drone1/camera/camera/color/image_raw', Image, process_image)

    print("Starting optimized lawnmower motion...")

    current_lat = home_lat
    current_lon = home_lon
    total_forward_progress = 0
    move_right = True  # Direction flag

    while total_forward_progress < max_distance and not target_found:
        # Calculate lateral movement direction (alternating left/right)
        lateral_angle = 90 if move_right else 270
        lateral_lat, lateral_lon = get_new_position(current_lat, current_lon, lateral_distance, lateral_angle)

        direction = "right" if move_right else "left"
        print("Moving " + direction + " to lat=" + str(lateral_lat) + ", lon=" + str(lateral_lon))

        goto(LocationGlobalRelative(lateral_lat, lateral_lon, takeoff_height))

        # Make forward progress
        current_lat, current_lon = get_new_position(lateral_lat, lateral_lon, forward_increment, 0)
        print("Moving forward to lat=" + str(current_lat) + ", lon=" + str(current_lon))
        goto(LocationGlobalRelative(current_lat, current_lon, takeoff_height))

        total_forward_progress += forward_increment
        move_right = not move_right  # Toggle direction for next pass

    print("Lawn Mower motion complete.")



# send location only when the target id is found
def send_marker_location(marker_id):
    """
    Sends the GPS location to the delivery drone when ID 72 is detected.
    """
    if marker_id == 72:
        print("Marker ID 72 found. Sending location to delivery drone.")
        marker_lat = vehicle.location.global_relative_frame.lat
        marker_lon = vehicle.location.global_relative_frame.lon
        marker_location = str(marker_lat) + "," + str(marker_lon)
        marker_location_pub.publish(marker_location)

        # Wait for acknowledgment
        while not ack_received:
            print("Waiting for acknowledgment from delivery drone...")
            marker_location_pub.publish(marker_location)
            time.sleep(1)

        print("Acknowledgment received. Continuing mission.")


def get_new_position(current_lat, current_lon, distance, bearing):
    current_lat_rad = math.radians(current_lat)
    bearing_rad = math.radians(bearing)
    R = 6378137.0  # Earth's radius in meters

    new_lat = current_lat + (distance / R) * math.degrees(math.cos(bearing_rad))
    new_lon = current_lon + (distance / R) * math.degrees(math.sin(bearing_rad) / math.cos(current_lat_rad))
    return new_lat, new_lon


if __name__ == "__main__":
    try:
        # Step 1: Save home location
        home_lat = vehicle.location.global_relative_frame.lat
        home_lon = vehicle.location.global_relative_frame.lon
        print("Home location saved: lat=" + str(home_lat) + ", lon=" + str(home_lon))

        # Step 2: Arm and takeoff
        arm_and_takeoff(takeoff_height)

        # Step 3: Perform zigzag motion
        print("Starting zigzag motion...")
        lawnmower_motion(lateral_distance=5, forward_increment=7, max_distance=25)

        # Step 4: Return to home location
        print("Returning to home location...")
        goto(LocationGlobalRelative(home_lat, home_lon, takeoff_height))

        # Step 5: Activate precision landing
        print("Activating precision landing...")
        if vehicle.mode != 'LAND':
            vehicle.mode = VehicleMode('LAND')
            while vehicle.mode != 'LAND':
                time.sleep(1)
            print('Vehicle is in LAND mode')

    except rospy.ROSInterruptException:
        print("ROS Interrupt. Mission aborted.")
    except KeyboardInterrupt:
        print("User aborted the mission.")
    except Exception as e:
        print("Unexpected error occurred: " + str(e))
    finally:
        vehicle.close()
