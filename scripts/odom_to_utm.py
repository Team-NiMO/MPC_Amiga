#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

class OdomProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odom_processor', anonymous=True)

        # Subscriber to the odometry topic
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # Publisher for the processed odometry data
        self.processed_odom_pub = rospy.Publisher('/odometry/filtered_utm', Odometry, queue_size=10)

        # TF2 listener to listen for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Placeholder for the latest odometry message
        self.latest_odom = None

    def odom_callback(self, msg):
        """
        Callback function for the odometry subscriber.
        Stores the latest odometry message.
        """
        self.latest_odom = msg
        self.process_odom()

    def process_odom(self):
        """
        Process the latest odometry message and publish the processed data.
        """
        if self.latest_odom is None:
            return

        try:
            # Lookup the transform
            transform = self.tf_buffer.lookup_transform('odom', 'utm', rospy.Time(0), rospy.Duration(1.0))

            # Transform the odometry pose
            odom_pose = self.latest_odom.pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(odom_pose, transform)

            # Create a new Odometry message with the transformed pose
            processed_odom = Odometry()
            processed_odom.header.stamp = rospy.Time.now()
            processed_odom.header.frame_id = 'odom'
            processed_odom.child_frame_id = 'base_link'
            processed_odom.pose.pose = transformed_pose.pose
            processed_odom.twist = self.latest_odom.twist

            # Publish the processed odometry message
            self.processed_odom_pub.publish(processed_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF2 Error: %s", e)

if __name__ == '__main__':
    try:
        odom_processor = OdomProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
