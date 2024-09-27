#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_ros

def publish_static_transform(parent_frame1, child_frame1, translation1, rotation1, parent_frame2, child_frame2, translation2, rotation2):
    rate = rospy.Rate(10000) 
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    while not rospy.is_shutdown():
        if rospy.get_param('/static_transform_broadcast/stop', False):
            rospy.loginfo("Stop signal received. Shutting down the static transform broadcaster.")
            break
        static_transformStamped_1 = TransformStamped()
        static_transformStamped_1.header.stamp = rospy.Time.now()
        static_transformStamped_1.header.frame_id = parent_frame1
        static_transformStamped_1.child_frame_id = child_frame1
        static_transformStamped_1.transform.translation.x = translation1[0]
        static_transformStamped_1.transform.translation.y = translation1[1]
        static_transformStamped_1.transform.translation.z = translation1[2]
        static_transformStamped_1.transform.rotation.x = rotation1[0]
        static_transformStamped_1.transform.rotation.y = rotation1[1]
        static_transformStamped_1.transform.rotation.z = rotation1[2]
        static_transformStamped_1.transform.rotation.w = rotation1[3]

        static_transformStamped_2 = TransformStamped()
        static_transformStamped_2.header.stamp = rospy.Time.now()
        static_transformStamped_2.header.frame_id =parent_frame2
        static_transformStamped_2.child_frame_id = child_frame2
        static_transformStamped_2.transform.translation.x = translation2[0]
        static_transformStamped_2.transform.translation.y = translation2[1]
        static_transformStamped_2.transform.translation.z = translation2[2]
        static_transformStamped_2.transform.rotation.x = rotation2[0]
        static_transformStamped_2.transform.rotation.y = rotation2[1]
        static_transformStamped_2.transform.rotation.z = rotation2[2]
        static_transformStamped_2.transform.rotation.w = rotation2[3]

        broadcaster.sendTransform([static_transformStamped_1, static_transformStamped_2])
    rospy.loginfo("Static transform published.")
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('broadcast_static_tranform', anonymous=True)
    rospy.set_param('/static_transform_broadcast/stop', False)
    publish_static_transform('world', 'map', [0, 0, 0], [0, 0, 0, 1], 'map', 'robot2/odom', [0, 0, 0], [0, 0, 0, 1])
