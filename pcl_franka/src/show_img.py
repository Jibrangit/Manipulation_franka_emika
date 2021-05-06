#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

img = np.load('/home/jibran_old/catkin_ws/src/Manipulation_franka_emika/pcl_franka/depth_img/depth_image.npy')

plt.imshow((img))
plt.show()
print(img.shape)






