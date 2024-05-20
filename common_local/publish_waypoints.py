#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

def publish_list_of_lists():
    # Initialize the ROS node
    rospy.init_node('list_of_lists_publisher', anonymous=True)

    # Create a publisher for the topic '/list_of_lists' with message type Float64MultiArray
    publisher = rospy.Publisher('/list_of_lists', Float64MultiArray, queue_size=10)

    # Define the loop rate (publishing interval) in Hz (e.g., 1 Hz means one message per second)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():
        # Create an instance of Float64MultiArray
        msg = Float64MultiArray()

        # Define the data
        data = [
            [6.26546496, 1.12449357],
            [10.66099288, 1.07280926],
            [14.70783303, 1.00785623],
            [19.86302758, 0.92477857]
        ]

        # Flatten the 2D list into a 1D list and assign it to msg.data
        msg.data = [element for sublist in data for element in sublist]

        # Publish the message
        publisher.publish(msg)

        # Log that the message has been published
        rospy.loginfo("Published list of lists message: %s", data)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    # Call the function to publish the list of lists
    try:
        publish_list_of_lists()
    except rospy.ROSInterruptException:
        pass