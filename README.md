# ROS 2 Image Publisher / Subscriber Demo

This project demonstrates a simple distributed system using ROS 2,
Python, and OpenCV.

Supported ROS 2 versions:
- Foxy Fitzroy
- Jazzy Jalisco

---

## 1. Requirements

Install ROS 2 Foxy or Jazzy:
https://docs.ros.org

Verify:
ros2 --version

Install Python dependencies:
pip install -r requirements.txt

---

## 2. Run the System

Terminal 1:
python publisher/image_publisher.py

Terminal 2:
python subscriber/image_subscriber.py

---

## 3. Output

Saved images:
output/images/

Metadata:
output/metadata.json