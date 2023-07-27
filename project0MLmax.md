
# Project 0: Applying Machine Learning Principles to ROS2
## Learning Objectives
- Practice writing ROS2 python scripts to control motors and sensors on the iRobot Create3
- Attach hardware to the Raspberry Pi and access data that can be used to determine the robot's actions
- Learn techniques in Machine Learning and use in code 

## Project Outline
This report outlines a more advanced project incorportating hardware and robotics concepts with Machine Learning. Students will use the iRobot Create3 with a Raspberry Pi Camera Module 3 and an ultrasonic sensor to navigate an obstacle course. Students will program robots to drive forward. If an object is within 6 inches of its path, students will take a photo using the Picamera2. Before running, students will use Google's Teachable Machine to create a trained data set on what 8 objects are but not their orientation or placement in the course. Thus, when they take a photo, the object will be identified accordingly. Prior to starting the final run through, students will be provided the course 10 minutes before, thus knowing whether to turn left or right after approaching the object. All turns should be 90 degrees.

## Resources needed
- iRobot Create3 robot
- Raspberry Pi 4
- Raspberry Pi Camera Module 3
- Google's [Teachable Machine](https://teachablemachine.withgoogle.com/train)
- Ultrasonic sensor (optional)

## Instructions

### Using the Raspberry Pi Camera Module 3 and creating the data set

To set up the Raspberry Pi Camera Module 3, lift up the flap of the J3 camera pin and insert the blue portion of the camera facing the audio check like so:

![Raspberry Pi 4 with Pi Camera Module 3 Inserted](j3installation.webp)

In order to be able to use the camera, we need to change the camera setting in the Raspberry Pi configuration page and download the package to be able to use the camera. First, download the camera package:

```
sudo apt install picamera2
```
Now, download the raspberry pi configuration:

```
sudo apt install raspi-config
```

Once this is installed, open the configuration by calling 
```
sudo raspi-config
```

Then, open interface options and disable Legacy Camera, and then reboot the pi


To test the camera, run the following python script:
```

```

Either using a code editor logged in to the pi or VNC viewer, this will allow you to see if the camera is taking photos, as you can view these photos in your desktop

Now that we know our camera is functioning, we can use it to train our Machine Learning Model.


### Training the Model
Google's Teachable Machine provides an intuitive first look at Machine Learning. 




