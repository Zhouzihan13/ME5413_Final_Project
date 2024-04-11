#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class GoalSequenceGenerator:
    def __init__(self):
        rospy.init_node('goal_sequence_generator')

        # Define three goal positions
        self.goal_positions = [
            (6.666267395019531, 1.5673103332519531, 0.002864837646484375),  # First position
            (7.325674057006836, -6.9588823318481445, 0.0068302154541015625),  # Second position
            (16.868064880371094, 2.221553325653076, 0.000762939453125)  # Third position
        ]

        # Define orientations for each goal position
        self.orientations = [1.0, 1.0, -1.0]

        # Initialize variables
        self.current_goal_index = 0
        self.goal_reached = False

        # Subscribe to move_base result to know when the robot reaches the goal
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_callback)

        # Create a publisher for the move_base_simple/goal topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Flag to indicate whether to continue with the sequence
        self.continue_sequence = True

    def result_callback(self, msg):
        # Check if the result status indicates success and the goal index is valid
        if msg.status.status == 3 and self.current_goal_index < len(self.goal_positions):
            # Update goal reached flag
            self.goal_reached = True

    def goal_callback(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))

    def generate_next_goal(self):
        # Check if all goals have been reached
        if self.current_goal_index >= len(self.goal_positions):
            rospy.loginfo("All goals have been reached.")
            return None

        # Create the next goal message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        x, y, z = self.goal_positions[self.current_goal_index]
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = self.orientations[self.current_goal_index]

        # Increment the goal index
        self.current_goal_index += 1

        return goal
    
    def objection_position_pose_callback(self, msg):
        # Check if objection_position_pose topic has received any message
        rospy.loginfo("Received message from objection_position_pose topic.")
        self.continue_sequence = False  # Stop the sequence

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Check if the current goal matches the one published by ObjectPositionPublisher
            if not self.continue_sequence:
                rospy.loginfo("Sequence stopped. Not generating new goals.")
                break

            # Check if the current goal has been reached
            if self.goal_reached:
                # Generate the next goal
                next_goal = self.generate_next_goal()
                if next_goal is not None:
                    # Publish the next goal
                    self.pub_goal.publish(next_goal)
                    rospy.loginfo("Published next goal.")
                    # Reset goal reached flag
                    self.goal_reached = False
                else:
                    rospy.loginfo("All goals have been reached or sequence stopped..")
            rate.sleep()

if __name__ == '__main__':
    try:
        goal_generator = GoalSequenceGenerator()
        # Subscribe to objection_position_pose topic to detect if any message is received
        rospy.Subscriber("/objection_position_pose", PoseStamped, goal_generator.objection_position_pose_callback)
        goal_generator.run()
    except rospy.ROSInterruptException:
        pass
