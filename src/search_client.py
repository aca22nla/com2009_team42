#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class ExplorerClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## TODO: get the current distance travelled, from the feedback message
        ## and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance travelled: {feedback_data.current_distance_travelled:.1f} meters. ")


    def __init__(self):
        self.distance = 0.0

        self.action_complete = False
        node_name = "search_action_client"
        action_server_name = "/search_action_server"
        rospy.init_node(node_name)
        self.rate = rospy.Rate(1)

        ## TODO: setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = actionlib.SimpleActionClient(action_server_name, 
                    SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## TODO: cancel the goal request, if this node is shutdown before the action has completed...
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        ## TODO: Print the result here...
        rospy.sleep(1) # wait for the result to come in
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * {self.distance} ... {self.client.get_result()}")

    def send_goal(self, fwd_vel, dist): 
        self.goal.fwd_velocity = fwd_vel
        self.goal.approach_distance = dist
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback) 

    def main(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.send_goal(fwd_vel= 0.05, dist = 0.5) 
        #debug
        print("Goal parameters set successfully:")
        print("Forward velocity:", self.goal.fwd_velocity)
        print("Approach distance:", self.goal.approach_distance)
        while self.client.get_state() < 2:
            print(f"STATE: Current state code is {self.client.get_state()}")
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance > 2:
                rospy.loginfo("More than 2 meters travelled. Cancelling Goal...")
                self.client.cancel_goal()
                self.action_complete = True 
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main() method from it...
    node = ExplorerClient()
    node.main()