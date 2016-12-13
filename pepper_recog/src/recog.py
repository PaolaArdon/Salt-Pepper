#!/usr/bin/env python

'''
Author@ Songyou Peng, Kaisar Kushibar and Paola Ardon, 2016

Framework@ Jose Maria Sola Duran, 2015
'''

from __future__ import print_function
import roslib
roslib.load_manifest('pepper_recog')
import sys
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from naoqi import ALProxy

from geometry_msgs.msg import Twist

import videoinput
import utilscv
import objrecogn as orec

from sensor_msgs.msg import Temperature 

class recognition():
    def __init__(self):

        self.bridge = CvBridge()

        self.motionPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.recog_pub = rospy.Publisher('recog', Temperature, queue_size=10)
        self.msg_recog = Temperature()
        self.msg_recog.variance = 0 
        self.msg_recog.temperature = 0.00
        self.msg_recog.header.frame_id = 'None'
        self.msg_recog.header.stamp = rospy.get_rostime();

        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

        self.drawKP = 0;
        self.videoinput = videoinput.VideoInput('0:ws=300:cols=400')
        self.paused = False
        self.dataBaseDictionary = orec.loadModelsFromDirectory()
        self.detector = cv2.SIFT(nfeatures=150)
        self.methodstr = 'SIFT'
        self.depthObj = 0
    def nothing(*arg):
        pass

    def callback(self,data):
        try: 
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        if (not self.paused) and (frame is not None):
            # self.curFrame = 0
            # convert color to gray
            imgin = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # output image for display
            imgout = frame.copy()

            t1 = time.time()
            kp, desc = self.detector.detectAndCompute(imgin, None)
            selectedDataBase = self.dataBaseDictionary[self.methodstr]
            if len(selectedDataBase) > 0:
                # Mutual Matching
                imgsMatchingMutuos = orec.findMatchingMutuosOptimizado(selectedDataBase, desc, kp)
                minInliers = int(20)
                projer = float(5)
                # Calculating the best image according to the inliers numbers
                # The best image is the one with the greater inliers number,
                # Always taking into account that it should exceed the minimum indicated in
                # the trackbar as 'minInliers' 
                bestImage, inliersWebCam, inliersDataBase =  orec.calculateBestImageByNumInliers(selectedDataBase, projer, minInliers)
                rate = rospy.Rate(10)
                motion = Twist();
                if not bestImage is None:
                    #If we find a good image, we calculate the affinity matrix and mark it on the screen as a recognized object
                    self.depthObj, center = orec.calculateAffinityMatrixAndDraw(bestImage, inliersDataBase, inliersWebCam, imgout)


                    self.msg_recog.variance = 1 
                    self.msg_recog.temperature = self.depthObj
                    self.msg_recog.header.frame_id = bestImage.nameFile.upper().split('.')[0]
                    self.msg_recog.header.stamp = rospy.get_rostime();

                    xc = 160
                    dx = xc - center[0]

                    motion.angular.z = dx/500.0;

                    if self.depthObj > 0.4:
                        motion.linear.x = 0.3
                    if self.depthObj < 0.2:
                        motion.linear.x = -0.3
                    
                    self.motionPub.publish(motion);
                else:
                    motion.linear.x = 0
                    self.motionPub.publish(motion);
                    rate.sleep();
                    self.depthObj = 0
                    self.msg_recog.variance = 0
                    self.msg_recog.temperature = 0.00
                    self.msg_recog.header.frame_id = 'None'
                    self.msg_recog.header.stamp = rospy.get_rostime();


            t1 = 1000 * (time.time() - t1)  # Time in milliseconds
            # Obtain the descriptors dimension in each features
            if desc is not None:
                if len(desc) > 0:
                    dim = len(desc[0])
                else:
                    dim = -1
            
            imgout = cv2.resize(imgout, (0, 0), fx = 1.45, fy = 1.45)
            # Drawing features, writing the informative text on the image
            # And, only draw features if the slider indicates it
            if self.drawKP > 0:
                cv2.drawKeypoints(imgout, kp, imgout, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            utilscv.draw_str(imgout, (20, 20),
                             "Method {0}, {1} features found ".
                             format(self.methodstr, len(kp)))
            utilscv.draw_str(imgout, (20, 40), "Time (ms): {0}".format(str(t1)))

            if self.depthObj == 0:
                utilscv.draw_str(imgout, (20, 60),
                                 "Depth to the object: None")
            else:
                utilscv.draw_str(imgout, (20, 60),
                             "Depth to the object: {0} m".
                             format(np.around(self.depthObj, decimals = 3)))

            # Show results and check the keyboards
            cv2.imshow('Pepper Object Recognition', imgout)
            ch = cv2.waitKey(5) & 0xFF
            if ch == ord(' '):  # Pause space bar
                self.paused = not self.paused
        self.recog_pub.publish(self.msg_recog)    
    # frame = cv2.resize(frame, (0, 0), fx = 3.0, fy = 3.0)



def main(args):
  rospy.init_node('recog', anonymous=True)
  ic = recognition()
  ROBOT_IP     = "192.168.150.7"
  ROBOT_PORT   = 9559
  proxy  = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
  proxy.say("Recognition mode is activated!");

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)