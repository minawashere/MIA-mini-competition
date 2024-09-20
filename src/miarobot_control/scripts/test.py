#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

current_point = Point()
setpoints = []
setpoint_index = 0
tolerance = 0.1
linear_speed_limit = 0.3
state = "MOVING"
direction = 1 

def generate_setpoints():
    max_y = 0.5
    step_x = 0.8
    step_y = 0.20
    start_x = 0.0

    global setpoints
    for y in [i * step_y for i in range(int(max_y / step_y) + 1)]:
        setpoints.append((start_x, y))
        setpoints.append((start_x + step_x, y))
        setpoints.append((start_x, y))
    
    for y in [-i * step_y for i in range(1, int(max_y / step_y) + 1)]:
        setpoints.append((start_x, y))
        setpoints.append((start_x + step_x, y))
        setpoints.append((start_x, y))

def odom_callback(msg):
    global current_point
    current_point = msg.pose.pose.position

def is_at_setpoint(target_x, target_y):
    distance = math.sqrt((target_x - current_point.x)**2 + (target_y - current_point.y)**2)
    return distance < tolerance

def move_to_next_setpoint(pub):
    global setpoint_index, state, direction

    target_x, target_y = setpoints[setpoint_index]

    if state == "MOVING":
        delta_x = target_x - current_point.x
        delta_y = target_y - current_point.y
        distance = math.sqrt(delta_x**2 + delta_y**2)

        if distance > 0:  # Prevent division by zero
            twist_msg = Twist()
            twist_msg.linear.x = min(linear_speed_limit, linear_speed_limit * (delta_x / distance))
            twist_msg.linear.y = min(linear_speed_limit, linear_speed_limit * (delta_y / distance))
            twist_msg.angular.z = 0.0

            pub.publish(twist_msg)

            if is_at_setpoint(target_x, target_y):
                state = "ARRIVED"
        else:
            state = "ARRIVED"  # Already at the setpoint

    elif state == "ARRIVED":
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        pub.publish(twist_msg)

        rospy.sleep(1)

        setpoint_index += direction
        if setpoint_index >= len(setpoints):
            direction = -1
            setpoint_index = len(setpoints) - 1
        elif setpoint_index < 0:
            direction = 1
            setpoint_index = 0

        state = "MOVING"

def main():
    rospy.init_node("robot_navigation_node")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    generate_setpoints()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        move_to_next_setpoint(pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
