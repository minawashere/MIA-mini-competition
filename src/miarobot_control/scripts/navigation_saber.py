#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations
import math

class Robot(object):
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        # Publishers
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Initialization
        self.current_position = [0, 0, 0]  # x, y, theta
        self.state = 0  # State machine
        self.start_time = rospy.get_time()

    def odom_callback(self, data):
        # Extract position
        self.current_position[0] = data.pose.pose.position.x
        self.current_position[1] = data.pose.pose.position.y

        # Extract quaternion to get direction
        quaternion = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]

        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_position[2] = euler[2]  # Yaw angle

    def set_velocities(self, linear_velocity, angular_velocity):
        velocity_message = Twist()
        velocity_message.linear.x = linear_velocity
        velocity_message.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_message)

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_time()

            if self.state == 0:  # Move forward
                self.set_velocities(0.2, 0)  # Move forward at 0.2 m/s
                if current_time - self.start_time >= 2:
                    self.start_time = current_time
                    self.state = 1

            elif self.state == 1:  # Turn right
                self.set_velocities(-0.2, -0.2)  # Turn right at 0.5 rad/s
                if current_time - self.start_time >= 2:
                    self.start_time = current_time
                    self.state = 2

            elif self.state == 2:  # Turn left
                self.set_velocities(0.2, 0.2)  # Turn left at 0.5 rad/s
                if current_time - self.start_time >= 2:
                    self.start_time = current_time
                    self.state = 3

            elif self.state == 3:  # Turn right again
                self.set_velocities(0.2, -0.2)  # Turn right at 0.5 rad/s
                if current_time - self.start_time >= 2:
                    self.start_time = current_time
                    self.state = 4

            elif self.state == 4:  # Move downward (assumed as a reverse motion)
                self.set_velocities(-0.2, 0.2)  # Move backward at 0.2 m/s
                if current_time - self.start_time >= 2:
                    self.start_time = current_time
                    self.state = 0  # Loop back to moving forward

            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        robot_instance = Robot()
        robot_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot motion interrupted.")