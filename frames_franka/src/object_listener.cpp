#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Point.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_wrt_world");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::Point IK_location;
    ros::Publisher IK_location_pub;
    IK_location_pub = nh.advertise<geometry_msgs::Point>("IK_location", 10);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try{
        transformStamped = tfBuffer.lookupTransform("world","object_frame", 
                                ros::Time(0));
        IK_location.x = transformStamped.transform.translation.x;
        IK_location.y = transformStamped.transform.translation.y;
        IK_location.z = transformStamped.transform.translation.z; 
        IK_location_pub.publish(IK_location);

        std::cout<<"\nThe object location in world is: \n";
        std::cout<<transformStamped.transform.translation.x<<std::endl;
        std::cout<<transformStamped.transform.translation.y<<std::endl;
        std::cout<<transformStamped.transform.translation.z<<std::endl;
        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        continue;
        }
        rate.sleep();
    }  
    return 0; 
}

