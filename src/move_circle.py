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

        self.direction = 1  # Direction of rotation: 1 for counterclockwise, -1 for clockwise
        self.circle_count = 0  # Count of completed circles
        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        self.ctrl_c = True

    def main(self):
        angular_speed = 1.0  # Angular speed in rad/s
        duration = 6.28 / angular_speed  # Time to complete one circle (2*pi / angular_speed)

        while not self.ctrl_c and self.circle_count < 2:  # Stop after completing one counterclockwise and one clockwise circle
            start_time = rospy.Time.now().to_sec()  # Get the current time
            while not self.ctrl_c:
                current_time = rospy.Time.now().to_sec()  # Get the current time
                elapsed_time = current_time - start_time  # Calculate elapsed time

                # Check if one full circle (360°) is completed
                if elapsed_time >= duration:
                    rospy.loginfo(f"One full circle completed in direction {'counterclockwise' if self.direction == 1 else 'clockwise'}.")
                    self.circle_count += 1  # Increment circle count
                    self.direction *= -1  # Reverse direction for the next circle
                    break  # Break out of the loop to switch direction

                # Publish movement commands
                move_cmd = Twist()
                move_cmd.linear.x = 0.5  # Linear velocity
                move_cmd.angular.z = angular_speed * self.direction  # Angular velocity with direction
                self.pub.publish(move_cmd)  # Publish movement command
                self.rate.sleep()

if __name__ == '__main__':
    circle = Circle()
    circle.main() 
