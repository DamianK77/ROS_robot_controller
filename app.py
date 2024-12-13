import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

class ROSMapVisualizer:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.latest_frame = None
        self.running = True

    def callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def subscriber(self):
        rospy.Subscriber(self.topic_name, Image, self.callback)

    def visualize(self):
        cv2.namedWindow("ROS Camera Feed", cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown() and self.running:
            if self.latest_frame is not None:
                # Display the latest frame
                cv2.imshow("ROS Camera Feed", self.latest_frame)
                key = cv2.waitKey(1)
                if key == 27:  # Press 'ESC' to exit
                    self.running = False
                    break
        cv2.destroyAllWindows()

    def start_visualization(self):
        rospy.init_node("ros_map_visualizer", anonymous=True)
        self.subscriber()
        visualization_thread = Thread(target=self.visualize)
        visualization_thread.start()
        rospy.spin()
        self.running = False
        visualization_thread.join()

if __name__ == "__main__":
    topic_name = input("Enter the topic name to subscribe to (e.g., '/camera/image_raw'): ")
    visualizer = ROSMapVisualizer(topic_name)
    try:
        visualizer.start_visualization()
    except rospy.ROSInterruptException:
        print("Shutting down ROS Map Visualizer")
