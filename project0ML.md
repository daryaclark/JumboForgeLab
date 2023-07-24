
# Project 0: Applying Machine Learning Principles to ROS2
## Learning Objectives
- Practice writing ROS2 python scripts to control motors and sensors on the iRobot Create3
- Attach hardware to the Raspberry Pi and access data that can be used to determine the robot's actions
- Learn techniques in Machine Learning and use in code 

## Project Outline
Students will use the iRobot Create3 with a Raspberry Pi Camera Module 3 and an ultrasonic sensor to navigate an obstacle course. Students will program robots to drive forward. If an object is within 6 inches of its path, students will take a photo using the Picamera2. Before running, students will use Google's Teachable Machine to create a trained data set on what 8 objects are but not their orientation or placement in the course. Thus, when they take a photo, the object will be identified accordingly. Prior to starting the final run through, students will be provided the course 10 minutes before, thus knowing whether to turn left or right after approaching the object. All turns should be 90 degrees.

## Resources needed
- iRobot Create3 robot
- Raspberry Pi 4
- Raspberry Pi Camera Module 3
- Google's [Teachable Machine](https://teachablemachine.withgoogle.com/train)
- Ultrasonic sensor (optional)

## Instructions

### Using the Raspberry Pi Camera Module 3 and creating the data set

To set up the Raspberry Pi Camera Module 3, lift up the flap of the J3 camera pin and insert the blue portion of the camera facing the audio check like so:




### Training the Model
Google's Teachable Machine provides an intuitive first look at Machine Learning. 




