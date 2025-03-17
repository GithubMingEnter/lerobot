#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2

class T265ImageSubscriber:
    def __init__(self):
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Subscribe to the T265 fisheye image topic
        self.image_subscriber = message_filters.Subscriber("/t265/fisheye2/image_raw", Image)

        # Subscribe to the pose topic (replace with your actual pose topic)
        self.pose_subscriber = message_filters.Subscriber("/arm1/rm_driver/Pose_State", Pose)

        # Synchronize the image and pose messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscriber, self.pose_subscriber],
            queue_size=30,
            slop=1,
            allow_headerless=True
        )
        self.sync.registerCallback(self.syncCallback)

    def syncCallback(self, image_msg, pose_msg):
        # Convert the ROS Image message to an OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Get the timestamp from the image message
        timestamp = image_msg.header.stamp.to_sec()
        rospy.loginfo(f"Received image with timestamp: {timestamp}")

        # Display the image using OpenCV
        cv2.imshow("T265 Fisheye Image", cv_image)
        cv2.waitKey(1)

        # Process the pose message (replace with your logic)
        rospy.loginfo(f"Received pose: {pose_msg}")

def main():
    # Initialize the ROS node
    rospy.init_node("t265_image_subscriber", anonymous=True)

    # Create an instance of the T265ImageSubscriber class
    t265_subscriber = T265ImageSubscriber()

    # Spin to keep the script running
    rospy.spin()

    # Clean up OpenCV windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()