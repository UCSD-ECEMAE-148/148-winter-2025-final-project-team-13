<div id="top"></div>

<h1 align="center"> SteeAir - Steer the car with hand in air!</h1>
<!-- PROJECT LOGO -->:
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="https://github.com/user-attachments/assets/717f342a-4328-457f-a119-9e82651d5955" alt="Logo" width="400" height="100">
  </a>
<h3>MAE148 Final Project</h3>
<p>
Team 13 Winter 2025
</p>

SteeAir is an innovative system that allows users to steer a car using hand gestures detected by a camera. This project leverages computer vision and machine learning techniques to provide intuitive and controller-free vehicle control. Additionally, a LiDAR-based security system is integrated for fail-safe operation and to correct unsafe driving behavior.

---

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware List](#hardware-list)
- [Implementation](#implementation)
- [Future Improvements](#future-improvements)

---

## Introduction

SteeAir uses camera input to recognize hand gestures, which are then translated into steering commands for controlling the car. The system also includes a LiDAR-based safety mechanism that acts as a fail-safe to ensure the vehicle operates safely even with potential errors in hand gesture recognition or unexpected driving behavior.

## Features

- **Hand Gesture Steering**: Control the vehicle with different hand gestures.
- **Real-Time Gesture Recognition**: Process hand gestures using an OAK-D camera to detect and translate them into steering actions.
- **LiDAR Integration**: A LiDAR-based system monitors the environment for obstacles and tracks driving behavior to provide fail-safe corrections.
- **Efficient Processing**: The hand gesture recognition model runs on the OAK-D camera, reducing the computational load on the Jetson Nano, ensuring smooth performance.
- **ROS2 Integration**: The system is integrated with ROS2 to handle communication between the hand gesture control and the LiDAR system.

## Hardware List

- **OAK-D Camera**: For hand gesture recognition and depth perception.
- **Jetson Nano**: For running the control system and machine learning models.
- **LiDAR Sensor**: For environment scanning and safety monitoring.

## Implementation

The hand gesture recognition model is designed to run directly on the OAK-D camera, ensuring that the computational load on the Jetson Nano is minimized. The system uses dockerized hand gesture classification model to capture hand gestures, mapping hem into steering commands for the vehicle.

### Hand Gesture Recognition

- The OAK-D camera captures video input of the user's hand gestures.
- A pre-trained machine learning model processes the images to identify specific hand gestures (one, two, fist, etc...)
- These gestures are mapped to corresponding steering actions, allowing users to control the vehicle by simply moving their hands.


https://github.com/user-attachments/assets/5adb9c2c-4ca9-4d20-9c99-be9ab224acd8


### LiDAR Safety System

- A LiDAR sensor is used to monitor the environment and detect obstacles in real-time.
- The system tracks the vehicle's surroundings and provides corrective actions if unsafe driving behavior is detected.
- If the system detects a potential collision or erratic steering, it can trigger the fail-safe mode to correct the vehicle's trajectory.

### ROS2 Integration

- The entire system is connected via ROS2, allowing seamless communication between the hand gesture recognition module and the LiDAR sensor.
- ROS2 facilitates data exchange between the components and enables smooth operation of the gesture-based steering system alongside the safety features.

## Future Improvements

- **Remote Video Input**: The current setup requires the driver to stand behind the vehicle, making steering less convenient. We aim to stream video data to the Jetson Nano so that the driver does not need to move and can control the vehicle from a more comfortable position.
  
- **Memory Option**: This issue can also be addressed by storing gestures as a series of commands or saving a drawn path in the air, allowing the system to recognize and repeat the gesture commands without requiring real-time input every time.
