#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist

class Circle():

    def __init__(self):
        self.node_name = "circle_mover"

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)

        self.ctrl_c = True

    def main(self):
        angular_speed = 1.0  # Angular speed in rad/s
        duration = 6.28 / angular_speed  # Time to complete one circle (2*pi / angular_speed)

        start_time = rospy.Time.now().to_sec()  # Get the current time
        while not self.ctrl_c:
            current_time = rospy.Time.now().to_sec()  # Get the current time
            elapsed_time = current_time - start_time  # Calculate elapsed time

            # Check if one full circle (360°) is completed
            if elapsed_time >= duration:
                rospy.loginfo("One full circle completed. Stopping the robot.")
                stop_cmd = Twist()  # Create a stop command
                self.pub.publish(stop_cmd)  # Publish the stop command
                break  # Break out of the loop

            # If the circle is not completed yet, continue publishing movement commands
            move_cmd = Twist()
            move_cmd.linear.x = 0.5  # Linear velocity
            move_cmd.angular.z = angular_speed  # Angular velocity
            self.pub.publish(move_cmd)  # Publish movement command
            self.rate.sleep()

if __name__ == '__main__':
    circle = Circle()
    circle.main() 
