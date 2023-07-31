# Project Example 1: Line Following Robot - Introduction to PID Controllers

## Learning Objectives
- Students will integrate a color sensor into the robot's design and learn how to calibrate and utilize the sensor to detect color variations on the track.
- Students will understand the principles of Proportional, Integral, and Derivative (PID) control and implement PID algorithms to accurately control the robot's movement along the line.
- Students will design, build, and assemble the physical structure of the robot and learn about the different components of a robot, including motors, sensors, and controllers, and how they work together to achieve specific tasks.

## Project Outline
Proportional, Integral, and Derivative (PID) control is a fundamental and widely used feedback control technique in engineering and robotics. The basic idea is to take information from a sensor and compute some output based on calculations for the three responses. The Proportional component calculates the error between the robot's position and the desired path, allowing us to determine the accuracy of our measurements.. The Integral calculations rely on measuring how successful previous changes have been.the Derivative component assesses how the error is changing over time. The PID controller sums these values for some output, which, in the case of the line folllower, is to adjust the steering of the robot accordingly. 

### Pseudocode 

In order to accomplish the above, the robot must:
1. Take a new light sensor reading
1. Compute the error
1. Scale error to determine contribution to steering update (proportional control)
1. Use error to update integral (sum of all past errors)
1. Scale integral to determine contribution to steering update (integral control)
1. Use error to update derivative (difference from last error)
1. Scale derivative to determine contribution to steering update (derivative control)
1. Combine P, I, and D feedback and steer robot
