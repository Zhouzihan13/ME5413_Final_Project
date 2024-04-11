#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def object_position_pose(t,o):
    #话题名为： objection_position_pose，
    #后面订阅这个话题时要使用这个名字
    pub = rospy.Publisher('/objection_position_pose',Pose,queue_size=10)
    p = Pose()
    rate = rospy.Rate(5)

    p.position.x = t[0]
    p.position.y = t[1]
    p.position.z = t[2]

    p.orientation.x = o[0]
    p.orientation.y = o[1]
    p.orientation.z = o[2]
    p.orientation.w = o[3]
    pub.publish(p)
    rate.sleep()

def callback(pose):
    object_position_info = pose.position
    object_orientation_info = pose.orientation
    object_position_pose([object_position_info.x, object_position_info.y, object_position_info.z],
                         [object_orientation_info.x, object_orientation_info.y, object_orientation_info.z, object_orientation_info.w])
    print(object_position_info)

def main():
    rospy.init_node('tf_listener',anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rospy.Subscriber('/objection_position_pose', Pose, callback)  # 添加订阅者
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/object_1', rospy.Time(0))
            print("trans:")
            print(trans)
            print("rot:")
            print(rot)
            object_position_pose(trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

# def main():
#     rospy.init_node('tf_listener', anonymous=True)
#     listener = tf.TransformListener()
#     rospy.Subscriber('/objection_position_pose', Pose, callback)  # 添加订阅者
#     rospy.spin()  # 让节点保持运行

if __name__ == '__main__':
    main()
