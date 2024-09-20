#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

linear_speed = 0.3
duration = 2
movement_sequence = []

def generate_movement_sequence():
    global movement_sequence
    movement_sequence = [
        (linear_speed, 0.0),  
        (-linear_speed, 0.0), 
        (0.0, linear_speed), 
        (linear_speed, 0.0),  
        (-linear_speed, 0.0), 
        (0.0, -linear_speed),
        (linear_speed, 0.0),  
        (-linear_speed, 0.0), 
        (0.0, -linear_speed),
        (linear_speed, 0.0),  
        (-linear_speed, 0.0), 
        (0.0, linear_speed)   
    ]

def move_robot(pub):
    global movement_sequence

    for linear_x, linear_y in movement_sequence:
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = 0.0
        pub.publish(twist_msg)

        rospy.sleep(duration)

        # Stop the robot after each movement
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        pub.publish(twist_msg)
        rospy.sleep(1)  # Optional pause before the next movement

def main():
    rospy.init_node("robot_movement_node")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    generate_movement_sequence()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        move_robot(pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
