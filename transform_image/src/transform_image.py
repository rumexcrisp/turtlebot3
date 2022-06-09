#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('transform_image')
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

VERBOSE=False

class image_converter:
    def __init__(self):
        self.image_pub_transform_compressed = rospy.Publisher("/fisheye_camera/image_transformed/compressed", CompressedImage, queue_size=1)
        self.image_pub_transform = rospy.Publisher("/fisheye_camera/image_transformed", Image, queue_size=1)
        self.image_pub = rospy.Publisher("/fisheye_camera/image_test/compressed", CompressedImage, queue_size=1)
        # self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/fisheye_camera/image_rect_color/compressed", CompressedImage, self.callback)
        # self.image_sub = rospy.Subscriber("/fisheye_camera/image_rect_color", Image, self.callback)

    def callback(self, data):
        # try:
        #     image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)

        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(cv_image, (50,50), 10, 255)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        lu = [9,379]
        # lo = [188,76]
        lo = [269,13]
        # ro = [388,65]
        ro = [373,14]
        # ru = [520,261]
        ru = [640,373]

        new_image_width = 38 * 6
        new_image_height = 38 * 16

        rows,cols,ch = image_np.shape
        # pts1 = np.float32([[180,rows-150],[cols-180,rows-150],[0,rows],[cols,rows]])
        pts1 = np.float32([lo,ro,lu,ru])
        pts2 = np.float32([[0,0],[new_image_width,0],[0,new_image_height],[new_image_width,new_image_height]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image_np,M,(new_image_width,new_image_height))

        cv2.circle(image_np,(int(lu[0]),int(lu[1])), 5, (0,0,255), -1) # lu
        cv2.circle(image_np,(int(lo[0]),int(lo[1])), 5, (0,0,255), -1) # lo
        cv2.circle(image_np,(int(ro[0]),int(ro[1])), 5, (0,0,255), -1) # ro
        cv2.circle(image_np,(int(ru[0]),int(ru[1])), 5, (0,0,255), -1) # ru

        # #### Feature detectors using CV2 #### 
        # # "","Grid","Pyramid" + 
        # # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        # method = "ORB"
        # # feat_det = cv2.FeatureDetector_create(method)
        # feat_det = cv2.ORB_create()
        # time1 = time.time()

        # # convert np image to grayscale
        # featPoints = feat_det.detect(cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        # time2 = time.time()
        # if VERBOSE :
        #     print('{} detector found: {} points in: {} sec.'.format(method, len(featPoints), time2-time1))

        # for featpoint in featPoints:
        #     x,y = featpoint.pt
        #     cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

        try:
            self.image_pub_transform.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))
        except CvBridgeError as e:
            print(e)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', dst)[1]).tostring()
        # Publish new image
        self.image_pub_transform_compressed.publish(msg)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
def main(args):
    ic = image_converter()
    rospy.init_node("transform_image", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
