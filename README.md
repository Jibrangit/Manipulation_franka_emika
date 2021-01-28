# Manipulation_franka_emika
Scripts of implementation of a manipulation pipeline in ROS Using Gazebo simualtor and using MOveit for motion planning. Only codes are included. 

Some of the codes are taken from 'Effective Robotics Programming with ROS' by Anil Mahtani and that is where I learnt how to play around with PCL. The scripts with 'Franka' in them are where the PCL processing for my project was done in the following sequence:

1. pcl_frank1.cpp takes in the point cloud from the Kinect camera and filters it(in this case, simply a threshold is applied to the camera). The threshold is applied in any direction(s) and it would be easy to find it in the code. This cloud is published into a node and is cubscribed to by pcl_segmentation_frank1.cpp

2. pcl_segmentation_frank1.cpp segments that cloud using RANSAC to get the major plane. Since, the filter cuts off the major plane due to the threshold(in this case), the table is detected. Further, the distance of this plane(table top) is extracted and published.

3. [NEXT UP] pcl_franka_cube.cpp subscribes to the original cloud(before segmentation and after filtering) and [YET TO BE DONE] creates a cloud of all points above the distance that was passed into this node. In this case, those points are the object on the table. 
