#!/usr/bin/env python
import rospy 
import numpy as np
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt

def imgCB( data ):
  im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
  np.save("/home/jibran_old/catkin_ws/src/Manipulation_franka_emika/pcl_franka/depth_img/depth_image.npy", im)
  print('Image saved!')


if __name__ == '__main__':
  rospy.init_node('img_object', anonymous=True)
  sub_vis = rospy.Subscriber('depth_img_object', numpy_msg(Image), imgCB)
  rospy.spin()