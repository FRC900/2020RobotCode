#!/usr/bin/env python

import rospy
import sys
import rospkg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

show_video = False

class Video2ROS:
    def __init__(self):
        rospy.init_node('pub_video', anonymous=False)
        rospy.on_shutdown(self.cleanup)

        self.input = "video.mp4"
        rospack = rospkg.RosPack()
        image_path = rospack.get_path('tf_object_detection') + '/src/'
        self.capture = cv2.VideoCapture(image_path + 'video.mp4')
        bridge = CvBridge()

        image_pub = rospy.Publisher("c920/rect_image", Image, queue_size=10)

        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except CvBridgeError, e:
                print e

            if show_video:
                display_image = frame.copy()
                cv2.imshow("Video Playback", display_image)

            self.keystroke = cv2.waitKey(25)

    def cleanup(self):
            print "Shutting down video pub node."
            cv2.destroyAllWindows()

def main(args):
    try:
        v2r = Video2ROS()
    except KeyboardInterrupt:
        v2r.cleanup()

if __name__ == '__main__':
    main(sys.argv)
