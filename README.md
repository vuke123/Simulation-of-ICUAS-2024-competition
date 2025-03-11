# UAV Localization System for Infrastructure Inspection | Research and Approach

## Overview
This project explores the development of an **aircraft localization system** for **infrastructure inspection**, with a particular focus on **vertical gardening**. The system is designed to enable **precise object localization, autonomous drone navigation, and plant detection** using advanced computer vision and robotics techniques.

A simulation environment has been developed in **Gazebo**, tailored for the **International Conference on Unmanned Aircraft Systems (ICUAS) 2024** competition. The implementation leverages **ROS (Robot Operating System)** and tools such as **Python, Pynput, OpenCV, and PyTorch** to facilitate accurate localization, control, and analysis.

## Features
- **UAV Localization:** Utilizes various localization techniques, including GPS-based and vision-based positioning.
- **Object Detection & Classification:** Employs the **YOLO v7 algorithm** for plant detection.
- **Simulated Environment:** Built within **Gazebo** for testing UAV movement, localization, and interaction.
- **Trajectory Planning:** Future implementation aims to integrate **voxelization** and **autonomous path planning**.
- **ROS Integration:** Uses **tf transformations** to map objects into a global coordinate system.
  
