#!/usr/bin/env python3

import roslaunch
import rospy

def main():
    rospy.init_node("map_builder", anonymous=True)
    rospy.loginfo("Map builder initialized successfully")

    # Create a ROS launch manager
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Define the file path for saving the map
    map_file_path = "$(find com2009_team42)/maps/task4_map"

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(0.2)  # 0.2 Hz corresponds to 5 seconds

    while not rospy.is_shutdown():
        rospy.loginfo(f"Saving file at time: {rospy.get_time()}...")

        # Define the map server node
        map_saver_node = roslaunch.core.Node(
            package="map_server",
            node_type="map_saver",
            output="screen",
            args=f"-f {map_file_path}",
        )

        # Launch the map server node
        process = launch.launch(map_saver_node)

        # Sleep to maintain the loop frequency
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
