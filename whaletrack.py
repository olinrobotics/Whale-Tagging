import rospy
import rospkg

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

class WhaleTracker:
    def __init__(self):
        #def attributes

        #init frame
        self.frame = None

        #initialize ros nod for whale detect, publish to whale location
        rospy.init_node('whale_detect', anonymous = True)
        self.pub = rospy.Publisher('/whale_location', String, queue_size=10)

        #CvBridge to usb_cam, subscribes to usb cam
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

    #converts ros message to numpy
    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def whale_track(self):
        if self.frame == None:
            prevframe = None
            return
        if prevframe == None:
            return

        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = 100,
                               qualityLevel = 0.3,
                               minDistance = 7,
                               blockSize = 7 )

        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (15,15),
                          maxLevel = 2,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Create some random colors
        color = np.random.randint(0,255,(100,3))

        old_gray = cv2.cvtColor(prevframe, cv2.COLOR_BGR2GRAY)
        p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

        # Create a mask image for drawing purposes
        mask = np.zeros_like(prevframe)

        #Optical flow
        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
