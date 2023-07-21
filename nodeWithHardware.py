# Title: nodeWithHardware.py
# Author: Darya Clark
# Purpose: Create a ROS2 node that uses hardware, in this case the RCWL-1601 
#          ultrasonic sensor 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# GPIO pins for the ultrasonic sensor
GPIO_TRIGGER = 18
GPIO_ECHO = 24

 # Initialize GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

class UltrasonicSensorNode(Node):
    # initialize node
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher = self.create_publisher(Float32, 'distance', 10)
        # repeat every second
        self.timer = self.create_timer(1, self.timer_callback)
        
    # call function and publish to logs
    def timer_callback(self):
        distance_cm = self.measure_distance()
        msg = Float32()
        msg.data = distance_cm
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    # use ultrasonic sensor to measure data
    def measure_distance(self):
        # Set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        start_time = time.time()
        stop_time = time.time()

        # Save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            start_time = time.time()

        # Save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            stop_time = time.time()

        # Time difference between start and arrival
        time_elapsed = stop_time - start_time

        # Speed of sound in air (343 meters per second) and 100 for conversion to centimeters
        distance_cm = round((time_elapsed * 34300) / 2, 2)

        print(distance_cm)

        return distance_cm

def main():
    rclpy.init()
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up resources
    finally: 
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
