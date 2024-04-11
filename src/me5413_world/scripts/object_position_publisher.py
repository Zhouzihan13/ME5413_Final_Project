#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ObjectPositionPublisher:
    def __init__(self):
        rospy.init_node('object_position_publisher')

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.objection_position_pub = rospy.Publisher('/objection_position_pose', PoseStamped, queue_size=10)  # 添加发布者
        self.objection_position_received = False

        # -------------------
        self.match_counter = 0  # Counter for consecutive matches
        self.adjust_interval = 3  # Consecutive matches interval in frames
        self.adjust_count = 0  # Counter for adjusting
        self.adjust_frequency = 5 # Adjust interval in frames
        # -------------------

        self.rate = rospy.Rate(10.0)
        rospy.Subscriber('/objection_position_pose', PoseStamped, self.callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

    def callback(self, pose):
        # if not self.objection_position_received:
        #     self.objection_position_received = True
        #     self.send_goal(pose)
        if not self.objection_position_received:
            self.objection_position_received = True
            self.match_counter += 1  # Match received, increment the counter
        else:
            self.match_counter = 0  # No match, reset the counter
        # if self.match_counter >= 5:  # Consecutive matches reached 6 frames
        #     self.send_goal(pose)
        if self.match_counter >= self.adjust_interval:  # Consecutive matches reached adjust interval
            self.match_counter = 0  # Reset the match counter
            self.send_goal(pose)
            self.adjust_count = 0  # Reset the adjust counter

    def send_goal(self, pose):
        
        processed_pose = self.process_goal(pose)

        try:
            # Publish the processed goal position message to the '/move_base_simple/goal' topic
            self.pub.publish(processed_pose)
            rospy.loginfo("Sending goal to move_base: {}".format(processed_pose))
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish goal: {}".format(str(e)))
        
        try:
            # Publish the original goal position message to the '/objection_position_pose' topic
            self.objection_position_pub.publish(pose)
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish original goal: {}".format(str(e)))
    
    def process_goal(self, pose):
        try:
            # Get the current position and orientation of the jackal
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x, y, z = trans[0], trans[1], trans[2]
            _, _, current_yaw = euler_from_quaternion(rot)

            # Get the original target position
            original_x = pose.pose.position.x
            original_y = pose.pose.position.y
            _, _, original_yaw = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
            ])

            # Calculate the Euler angles of the target position, and adjust to the nearest ±90 degrees direction to the current orientation
            target_yaw = current_yaw + math.copysign(math.pi / 2.0, current_yaw)

            target_x = original_x + 1 * math.cos(original_yaw)
            target_y = original_y + 1 * math.sin(original_yaw)

            # Construct the quaternion representation of the adjusted target position
            quat = quaternion_from_euler(0, 0, target_yaw)

            # Create the processed goal point message
            processed_pose = PoseStamped()
            processed_pose.header.stamp = rospy.Time.now()
            processed_pose.header.frame_id = pose.header.frame_id  # Same coordinate system as the original target position
            processed_pose.pose.position.x = target_x
            processed_pose.pose.position.y = target_y
            processed_pose.pose.position.z = 0  # Set z coordinate to 0
            processed_pose.pose.orientation = Quaternion(*quat)

            return processed_pose

        except rospy.ROSException as e:
            rospy.logerr("Failed to process goal: {}".format(str(e)))


    def goal_callback(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))
    

    def adjust_goal(self):
        if self.adjust_counter >= self.adjust_frequency:
            if self.objection_position_received:
                self.send_goal(self.current_goal)
            else:
                rospy.loginfo("No objection position detected. Maintaining current goal.")
            self.adjust_counter = 0


    def run(self):
        while not rospy.is_shutdown():
            try:
                if not self.objection_position_received:
                    (trans, rot) = self.listener.lookupTransform('/map', '/object_1', rospy.Time(0))
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = trans[0]
                    pose.pose.position.y = trans[1]
                    pose.pose.position.z = trans[2]
                    pose.pose.orientation.x = rot[0]
                    pose.pose.orientation.y = rot[1]
                    pose.pose.orientation.z = rot[2]
                    pose.pose.orientation.w = rot[3]
                    self.send_goal(pose)
                    self.objection_position_received = True
                elif self.current_goal is not None:
                    # Adjust the current goal pose while the objection position is received
                    self.adjust_goal()
                else:
                    rospy.loginfo("No current goal. Waiting for the next objection position.")
                    self.objection_position_received = False  # Reset the flag to wait for the next objection position

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to lookup obstacle position")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObjectPositionPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
