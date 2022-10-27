import rospy

import cv2
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import sys

import numpy as np

class image_converter:

    def __init__(self):
        """
        This is a node that subscribes to the RGB image topic of rosout, 
        convert each sensor_msg.msg.Image message at each timestamp to cv2 compatible numpy.ndarray
        """
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.depth_pub = rospy.Publisher("depth_topic_2",Image)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/airsim_node/PX4/camera_1/Scene',Image,self.callback)
        # self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        # self.depth_sub = rospy.Subscriber('/camera/depth/duy/image_raw', Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber('/camera_front/depth/image_raw', Image, self.depth_frame_id_callback)

    def depth_callback(self,msg):
        try:
            #(Maxvalue: 65535)
            # Equivalent to
            # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            h, w = msg.height, msg.width
            dtype = np.dtype("uint16")
            cv_image = np.ndarray(shape=(h, w),
                           dtype=dtype, buffer=msg.data)  

        except CvBridgeError as e:
            print(e)

        (rows,cols) = cv_image.shape
        #convert to float32 (Maxvalue: 65535.0)
        cv_image = np.asarray(cv_image, dtype=np.float32)

        #convert to m:
        cv_image = cv_image / 1000.

        cv2.imshow("Depth Image window", cv_image)
        print(cv_image.max())
        cv2.waitKey(3)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "32FC1")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera_link"
            self.depth_pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)
        
    # DuyNguyen
    def depth_frame_id_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1") 

        except CvBridgeError as e:
            print(e)
        # //A Minh
        viz = cv_image
        viz[viz>1000] = 1000
        
        cv2.imshow("Depth Image window", viz)
        print(cv_image.max())
        cv2.waitKey(3)

        try:
            # convert frame_id to "camera_link"
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "32FC1")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera_link"
            self.depth_pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    #Step 1: Initialize Python3 Object
    ic = image_converter()
    #Step 2: Initialize ROS node
    rospy.init_node('image_converter', anonymous=True)
    #Step 3: Run
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #Step 4: Finish
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
