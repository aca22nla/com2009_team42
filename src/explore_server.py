#! /usr/bin/env python3
# search_server.py

import rospy
import actionlib
import waffle
import math


from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal



class RobotExplorer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.server_name = "search_action_server"
        rospy.init_node(self.server_name)

        # functions from the waffle.py module:
        self.motion = waffle.Motion(debug = True)
        self.pose = waffle.Pose(debug = True)
        self.lidar = waffle.Lidar(debug = True)
        rospy.loginfo("Lidar initialized successfully")

        self.actionserver = actionlib.SimpleActionServer(self.server_name, SearchAction,
                                                          self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.min_front_distance = 0.4
        self.min_side_distance = 0.25
        self.max_angular_velocity = math.radians(35)  # Maximum angular velocity in radians per second

        rospy.loginfo("The 'Search Action Server' is active...")

    
    def calculate_angular_velocity(self, closest_object_location):
        orientation_difference = closest_object_location - self.pose.yaw
        # Normalize the orientation difference to the range [-pi, pi]
        orientation_difference = (orientation_difference + math.pi) % (2 * math.pi) - math.pi
        
        # Calculate angular velocity based on the orientation difference
        return orientation_difference  # Adjust the coefficient as needed


            
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
            self.result.total_distance_travelled = 0
            self.actionserver.set_aborted(SearchResult())
            return
        #if requested goal is valid
        print(f"Valid goal. Continue action...")


        while not rospy.is_shutdown():
            # Get the robot's current odometry
            posx0 = self.pose.posx
            posy0 = self.pose.posy

            # Check LiDAR data
            self.closest_object = min(self.lidar.subsets.frontArray)
            self.right_obstacle = min(self.lidar.subsets.l1, self.lidar.subsets.l2)
            self.left_obstacle = min(self.lidar.subsets.r1, self.lidar.subsets.r2)

            # self.closest_object_location = self.tb3_lidar.closest_object_position #angle

            #check if there is object detected
            if self.closest_object <= self.min_front_distance:
                rospy.loginfo("Obstacle detected. Avoiding obstacle")

                # Calculate angular velocity to avoid the obstacle
                min_distance_index = self.lidar.subsets.frontArray.index(self.closest_object)
                angle_increment = 0.5
                closest_object_location = (min_distance_index - len(self.lidar.subsets.frontArray) / 2) * angle_increment
                angular_velocity = self.calculate_angular_velocity(self.closest_object)
                self.motion.set_velocity(linear=0.0, angular=angular_velocity)
                self.motion.publish_velocity()

                rospy.sleep(1)  # Allow the robot to turn for 1 second

            # if self.left_obstacle <= self.min_side_distance :
            #     rospy.loginfo("Obstacle detected. Avoiding obstacle")


            #     angular_velocity = self.calculate_angular_velocity(self.left_obstacle)
            #     self.motion.set_velocity(linear=0.0, angular=angular_velocity)
            #     self.motion.publish_velocity()

            #     rospy.sleep(1)  # Allow the robot to turn for 1 second



            else:
                # No obstacle detected, continue exploration
                self.motion.set_velocity(linear=0.2, angular=0.0)
                self.motion.publish_velocity()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Action preempted. Cancelling goal...")
                self.actionserver.set_preempted(self.result)
                # Stop the robot
                self.motion.stop()
                success = False
                # exit the loop:
                break


            # determine how far the robot has travelled so far:
            self.distance = math.sqrt(pow(posx0 - self.pose.posx, 2) + pow(posy0 - self.pose.posy, 2))

            # update feedback message values and publish feedback:
            self.feedback.current_distance_travelled = self.distance
            rospy.loginfo(f"Travelled {self.feedback.current_distance_travelled:.2f} m")
            self.actionserver.publish_feedback(self.feedback)

            # update all result parameters:
            self.result.total_distance_travelled = self.distance
            # self.result.closest_object_angle = self.closest_object_location
            # self.result.closest_object_distance = self.closest_object

            rate.sleep()
            
        if success:
            rospy.loginfo("approach completed successfully.")
            self.actionserver.set_succeeded(self.result)
            self.motion.stop()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = RobotExplorer()
    node.main()
