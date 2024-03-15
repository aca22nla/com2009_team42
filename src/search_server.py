#! /usr/bin/env python3
# search_server.py

import rospy
import actionlib


from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt, pow, radians


class RobotExplorer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.server_name = "search_action_server"
        rospy.init_node(self.server_name)

        self.actionserver = actionlib.SimpleActionServer(self.server_name, SearchAction,
                                                          self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.min_safe_distance = 0.5
        self.max_angular_velocity = radians(45)  # Maximum angular velocity in radians per second
        self.turn_threshold = radians(10)  # Angle threshold for initiating a turn

        rospy.loginfo("The 'Search Action Server' is active...")

    def calculate_angular_velocity(self):
        # Calculate angular velocity based on laser scan data
        if self.closest_object > self.min_safe_distance:
            # No obstacles nearby, move forward
            return 0.0
        elif abs(self.closest_object_location) < self.turn_threshold:
            # Obstacle detected near the front, initiate a turn
            return self.max_angular_velocity
        else:
            # Obstacle detected, calculate angular velocity to follow the gap
            angular_velocity = (self.closest_object_location - 0) / 20.0  # proportional control
            return angular_velocity

            
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        # Print the received goal for debugging
        rospy.loginfo(f"Received goal: fwd_velocity={goal.fwd_velocity}, approach_distance={goal.approach_distance}")

        # Goal input parameter(s)
        success = True
        if goal.fwd_velocity > 0.26:
            rospy.logerr("Too fast! The robot's max velocity is 0.26 m/s.")
            print("Too fast! The robot's max velocity is 0.26 m/s.")
            success = False

        if goal.approach_distance < 0.25:
            print("Too near! The robot may hit an obstacle.")
            rospy.logerr("Too near! The robot may hit an obstacle.")
            success = False


        if not success:
            # abort the action server if an invalid goal has been requested...
            rospy.logerr("Aborting action due to invalid goal...")
            self.result.closest_object_distance = 0
            self.result.total_distance_travelled = 0
            self.result.closest_object_angle = 0
            self.actionserver.set_aborted(self.result)
            return
        #if requested goal is valid
        print(f"Valid goal. Continue action...")


        while not rospy.is_shutdown():
            # Get the robot's current odometry from the Tb3Odometry() class:
            self.posx0 = self.tb3_odom.posx
            self.posy0 = self.tb3_odom.posy
            # Get information about objects up ahead from the Tb3LaserScan() class:
            self.closest_object = self.tb3_lidar.min_distance #distance
            self.closest_object_location = self.tb3_lidar.closest_object_position #angle
            
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Action preempted. Cancelling goal...")
                # Stop the robot
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

            if self.closest_object <= self.min_safe_distance:
                rospy.loginfo("Obstacle detected. Avoiding obstacle.")

                # Stop the robot
                self.vel_controller.stop()
                rospy.sleep(1)  # Pause for 1 second

                # Calculate angular velocity to avoid the obstacle
                angular_velocity = -1.0  # Turn in the opposite direction
                self.vel_controller.set_move_cmd(0.0, angular_velocity)
                self.vel_controller.publish()

                rospy.sleep(1)  # Turn for 2 seconds

            else:
                # no obstacle detected, continue exploration
                angular_velocity = self.calculate_angular_velocity()
                # Set angular velocity to navigate the robot
                self.vel_controller.set_move_cmd(0.1, angular_velocity)
                # Publish velocity command
                self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            # update feedback message values and publish feedback:
            rospy.loginfo(f"Travelled {self.feedback.current_distance_travelled}m")
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            # update all result parameters:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.result.closest_object_distance = self.closest_object

            rate.sleep()
            
        if success:
            rospy.loginfo("approach completed successfully.")
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = RobotExplorer()
    node.main()
