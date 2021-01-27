#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler 
{
public:
    cloudHandler() 
    {
        base_sub = nh.subscribe("base_location", 10, &cloudHandler::BaseLocationCB, this);
        obj_pub  = nh.advertise<sensor_msgs::PointCloud2>("pcl_object", 1);
        pcl_obj_sub = nh.subscribe("pcl_franka_filtered", 10, &cloudHandler::cloudCB, this);
    }

    void BaseLocationCB(std_msgs::Float32 &base_location)
    {
        z_base = base_location;
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> object_cloud;


        pcl::fromROSMsg(input, cloud);

        size_t j{};
        for(size_t i{}; i<cloud.size(); ++i) 
        {
            if(cloud.points[i].y < z_base)
            {
                object_cloud.points[j].x = cloud.points[i].x;
                object_cloud.points[j].y = cloud.points[i].y;
                object_cloud.points[j].z = cloud.points[i].z;
                ++j;
                std::cout<<"Point added"<<std::endl;
            }

        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(object_cloud, output);
        obj_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_obj_sub, base_sub;
    ros::Publisher obj_pub, base_location;
    std_msgs::Float32 z_base;
};

main(int argc, char **argv) 
{
    ros::init(argc, argv, "pcl_franka_cube");
    cloudHandler handler;
    ros::spin();
    return 0;
}