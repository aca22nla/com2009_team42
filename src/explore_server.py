#! /usr/bin/env python3
# search_server.py

import rospy
import waffle
import math
import roslaunch
from testcolour_search import TestColourSearch

class RobotExplorer():
    
    def __init__(self):
        self.node = "explore_server"
        rospy.init_node(self.node)

        self.rate = rospy.Rate(10)
        self.target_colour = rospy.get_param("~target_colour", "")
        self.colour_search = None
        if self.target_colour in ["yellow", "red", "green", "blue"]:
            self.colour_search = TestColourSearch(self.target_colour)
        
        # functions from the waffle.py module:
        self.motion = waffle.Motion(debug = True)
        self.pose = waffle.Pose(debug = True)
        self.lidar = waffle.Lidar(debug = True)
        rospy.loginfo("Lidar initialized successfully")


        self.min_front_distance = 0.4
        self.min_side_distance = 0.25
        self.max_angular_velocity = math.radians(35)  # Maximum angular velocity in radians per second

        rospy.loginfo("The 'Explore Server' is active...")

    def calculate_angular_velocity(self, closest_object_location):
        orientation_difference = closest_object_location - self.pose.yaw
        # Normalize the orientation difference to the range [-pi, pi]
        orientation_difference = (orientation_difference + math.pi) % (2 * math.pi) - math.pi
        
        # Define the maximum allowed angular velocities
        pmax_angular_velocity = 1.6  # Maximum angular velocity in radians per second
        nmax_angular_velocity = -1.6  # Minimum angular velocity in radians per second
        
        # Calculate angular velocity based on the orientation difference
        angular_velocity = orientation_difference
        
        # Limit angular velocity to the maximum allowed value
        if angular_velocity > pmax_angular_velocity:
            angular_velocity = pmax_angular_velocity
        elif angular_velocity < nmax_angular_velocity:
            angular_velocity = nmax_angular_velocity

        return angular_velocity 


    # def save_map_periodically(self):

    #     print(f"Saving file at time: {rospy.get_time()}...")

    #     node = roslaunch.core.Node(
    #         package="map_server",
    #         node_type="map_saver",
    #         output="screen",
    #         args=f"-f $(find com2009_team42)/maps/task4_map",
    #     )
    #     launch = roslaunch.scriptapi.ROSLaunch()
    #     launch.start()
    #     process = launch.launch(node)

    def main(self):

        while not rospy.is_shutdown():

            # Get the robot's current odometry
            posx0 = self.pose.posx
            posy0 = self.pose.posy

            # Check LiDAR data
            self.closest_object = min(self.lidar.subsets.frontArray)
            self.right_obstacle = min(self.lidar.subsets.l1, self.lidar.subsets.l2)
            self.left_obstacle = min(self.lidar.subsets.r1, self.lidar.subsets.r2)

            if self.target_colour and self.colour_search:

            #check if there is object detected
                if self.closest_object <= self.min_front_distance:
                    rospy.loginfo("Obstacle detected. Avoiding obstacle")

                    # Calculate angular velocity to avoid the obstacle
                    angular_velocity = self.calculate_angular_velocity(self.closest_object)
                    self.motion.set_velocity(linear=0.0, angular=angular_velocity)
                    self.motion.publish_velocity()

                    rospy.sleep(1)  # Allow the robot to turn for 1 second

                else:
                    # No obstacle detected, continue exploration
                    self.motion.set_velocity(linear=0.2, angular=0.0)
                    self.motion.publish_velocity()

                img_data = self.colour_search.get_latest_img_data()
                self.colour_search.camera_callback(img_data) # To perform colour detection

            else:

                if self.closest_object <= self.min_front_distance:
                    rospy.loginfo("Obstacle detected. Avoiding obstacle")

                    # Calculate angular velocity to avoid the obstacle
                    angular_velocity = self.calculate_angular_velocity(self.closest_object)
                    self.motion.set_velocity(linear=0.0, angular=angular_velocity)
                    self.motion.publish_velocity()

                    rospy.sleep(1)  # Allow the robot to turn for 1 second
                else:
                    # No obstacle detected, continue exploration
                    self.motion.set_velocity(linear=0.2, angular=0.0)
                    self.motion.publish_velocity()

            posx1 = self.pose.posx
            posy1 = self.pose.posy
            # determine how far the robot has travelled so far:
            self.distance = math.sqrt(pow(posx1 - posx0, 2) + pow(posy1 - posy0, 2))

            posx0, posy0 = posx1, posy1

            # if img_data is None:
            #     rospy.logerr("No image data available")   
            # else:
            #     img_data
                 
            self.rate.sleep()


if __name__ == '__main__':
    # rospy.init_node("search_action_server")
    node = RobotExplorer()
    node.main()
    rospy.spin()
