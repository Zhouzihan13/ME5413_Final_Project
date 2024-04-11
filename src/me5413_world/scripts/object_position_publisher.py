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
        self.match_counter = 0  # 连续匹配计数器
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
            self.match_counter += 1  # 收到匹配，计数器加1
        else:
            self.match_counter = 0  # 没有匹配，重置计数器
        if self.match_counter >= 2:  # 连续匹配达到3帧
            self.send_goal(pose)

    def send_goal(self, pose):
        
        processed_pose = self.process_goal(pose)

        try:
            # 发布处理后的目标位置消息到 /move_base_simple/goal 话题上
            self.pub.publish(processed_pose)
            rospy.loginfo("Sending goal to move_base: {}".format(processed_pose))
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish goal: {}".format(str(e)))
        
        try:
            # 发布原始的目标位置消息到 /objection_position_pose 话题上
            self.objection_position_pub.publish(pose)
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish original goal: {}".format(str(e)))
    
    def process_goal(self, pose):
        try:
            # 获取机器人当前位置和朝向
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x, y, z = trans[0], trans[1], trans[2]
            _, _, current_yaw = euler_from_quaternion(rot)

            # 获取原始目标点位置
            original_x = pose.pose.position.x
            original_y = pose.pose.position.y
            _, _, original_yaw = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
            ])

            # 计算目标位置的欧拉角，并调整为与当前方向最接近的正负90度朝向
            target_yaw = current_yaw + math.copysign(math.pi / 2.0, current_yaw)

            target_x = original_x + 1 * math.cos(original_yaw)
            target_y = original_y + 1 * math.sin(original_yaw)

            # 构造调整后的目标位置的四元数表示
            quat = quaternion_from_euler(0, 0, target_yaw)

            # 创建处理后的目标点消息
            processed_pose = PoseStamped()
            processed_pose.header.stamp = rospy.Time.now()
            processed_pose.header.frame_id = pose.header.frame_id  # 与原始目标位置相同的坐标系
            processed_pose.pose.position.x = target_x
            processed_pose.pose.position.y = target_y
            processed_pose.pose.position.z = 0  # z 坐标设为 0
            processed_pose.pose.orientation = Quaternion(*quat)

            return processed_pose

        except rospy.ROSException as e:
            rospy.logerr("Failed to process goal: {}".format(str(e)))


    def goal_callback(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))

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
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to lookup obstacle position")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObjectPositionPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
