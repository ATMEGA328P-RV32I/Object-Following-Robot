# Raspberry Pi Vision Tracker Robot

An autonomous Raspberry Pi-based robot that uses OpenCV and Picamera2 to track a visual target while avoiding obstacles.

Board used - Raspberry Pi 4B

camera - Raspberry Pi 5MP camera

Motor driver - L298N

This project features multi-scale template matching (so the robot recognizes the target as it gets closer/further) and adaptive lighting (automatically applying CLAHE to see into shadows when the environment gets dark).

## Features
* **Vision Tracking:** Uses OpenCV `matchTemplate` to find and follow a target image.
* **Distance Invariance:** Checks multiple scales (0.5x, 1.0x, 1.5x) to maintain a lock on the target at varying distances.
* **Adaptive Lighting:** Automatically detects low-light environments and applies a CLAHE filter to prevent losing the target in the dark.
* **Proportional Control:** Smooth steering and variable speed logic—slows down for sharp turns and stops precisely when close to the target.
* **Obstacle Avoidance:** Uses an HC-SR04 ultrasonic sensor to trigger an emergency stop if an object is closer than 25cm.

## Hardware & Pinout
This code relies on the `pigpio` factory for hardware PWM and precise timing. Wire your Raspberry Pi to your motor driver and ultrasonic sensor using the following BCM GPIO pins:

### Motors
* **Left Motor:** FWD = `GPIO 17`, BWD = `GPIO 18`, Enable = `GPIO 27`
* **Right Motor:** FWD = `GPIO 22`, BWD = `GPIO 23`, Enable = `GPIO 24`

### Ultrasonic Sensor (HC-SR04)
* **Trigger:** `GPIO 5`
* **Echo:** `GPIO 6`

## Repository Structure
Ensure your files are structured like this before running the code. The `targets/box.jpg` file is mandatory!

```text
├── targets/               
│   └── box.jpg            <-- The image the robot will look for
├── main.py                <-- Main robot script
├── requirements.txt       <-- Python pip dependencies
└── README.md
```

## Installation and Setup
1. Install System-Level Dependencies
picamera2 and pigpio require system-level installation on Raspberry Pi OS. Run this in your terminal:
```
sudo apt update
sudo apt install python3-picamera2 python3-pigpio
```
2. Enable and Start the PiGPIO Daemon
The motor controls and ultrasonic sensor rely on the pigpiod service. Enable it so it runs in the background:
```
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```
3. Install Python Libraries
Install the remaining computer vision and math libraries:
```
pip install -r requirements.txt
```
## How to Run
Once everything is wired and installed, place your target image (box.jpg) in the targets folder and run the script:
```
python3 main.py
```
Press Ctrl+C to stop the robot and exit the script
