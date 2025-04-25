# 🤖 Autonomous Multi-Vehicle System

This project is all about getting multiple drones (or a drone and a ground vehicle) to work together on a shared task — fully autonomously. One vehicle acts as a scout, using computer vision to find a target. The other receives that info and performs a delivery or precision landing at the right location.

---

## What It Does

- One drone (the **scout**) searches an area in a **lawnmower (zigzag)** pattern, looking for special ArUco markers.
- When it finds a marker, it sends the location to another vehicle (the **delivery unit**).
- The delivery vehicle then flies to the given location to drop off a payload or land accurately.
- The system includes basic communication checks: if the delivery vehicle doesn't respond, the scout keeps trying.

---

## 🔑 Key Features

- **Lawnmower Search Pattern**: Scout drone systematically covers the area, moving side to side and forward like mowing a lawn.
- **ArUco Marker Detection**: Uses computer vision to find and identify markers like “home” or “delivery point”.
- **Two-Way Communication**: Vehicles talk to each other using DroneKit and MAVLink.
- **Precision Landing**: Delivery drone uses marker detection to land on a specific target.
- **Mission Logging**: Logs when markers are found, messages are sent, and missions are completed.

---

## 🧰 Tech Stack

- **Python**
- **OpenCV** for computer vision and marker detection
- **DroneKit / MAVLink** for communication
- **Raspberry Pi + Pixhawk**
- **Gazebo + ROS** (for simulation testing)
