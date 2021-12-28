#!/usr/bin/env python3
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
if '/opt/ros/melodic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
    sys.path.append('/home/hao/cv_bridge/install/lib/python3/dist-packages')
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ORBextractor:

  def __init__(self):
    self.image_pub = rospy.Publisher("/ORBextractor/image",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    orb = cv2.ORB_create(1000)
    kp, des = orb.detectAndCompute(cv_image, None)
    outimg = cv2.drawKeypoints(cv_image, kp, None, (0,255,255), 0)
    print("keypoints = {0}\n".format(kp))

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(outimg, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = ORBextractor()
  rospy.init_node('ORBextractor', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)