# Face-Tracking-Camera 📸 

A robotic project that uses the Raspberry Pi 5 and Python to build and code a pan and tilt camera that tracks human faces.

## Demo Video

!!click the link to watch video!! [https://youtu.be/XimNa6xnBiA](https://youtu.be/YztODTyrzEg)


## Project Overview 🤖

### Software

* written in Python
* OpenCV for facial recognition
* Active loop used to assess current frame and detect faces
* movement of the servo motor angles is determined by mathematical parameters/thresholds
* face value tracking to handle focusing on one face when multiple are detected

### Hardware

* Raspberry Pi 5 is the computer
* two metal servo motors (MG90S 9 gram) for panning and tilting the camera
* hardware timed Pulse Width Modulation (PWM) board (PCA9685) for controlling the servo motors
* Raspberry Pi camera module
* used cardboard, paperclips, rubberbands, and tape to create a makeshift camera stand and stabalize the setup
* old IPad charger used as the 5V external power source for the PWM board


## Accomplished

* successful face-tracking camera that is very responsive, accuarate, and handles tracking with multiple faces
* learned about GPIO programming and PWM servo motors
* used a machine learning model for a real world application
* gained experience in software-hardware integration and troubleshooting


## Tips For Others

* use hardware timed PWM for much smoother servo movement
* use metal servos to avoid the servos breaking from the load of the camera/motors
