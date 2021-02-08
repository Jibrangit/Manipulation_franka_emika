#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        base_sub = nh.subscribe("base_location", 10, &cloudHandler::BaseLocationCB, this);
        pcl_obj_sub = nh.subscribe("pcl_franka_filtered", 10, &cloudHandler::cloudCB, this);
        obj_pub  = nh.advertise<sensor_msgs::PointCloud2>("pcl_object", 1);
        centroid_pub = nh.advertise<std_msgs::Float32MultiArray>("object_centroid", 1);
    }

    void BaseLocationCB(const std_msgs::Float32 &base_location)
    {
        z_base.data = base_location.data;
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> object_cloud;
        sensor_msgs::PointCloud2 object_output;

        pcl::fromROSMsg(input, cloud);

        object_cloud.width = cloud.width;
        object_cloud.height = cloud.height;
        object_cloud.resize(object_cloud.width * object_cloud.height);

        size_t j{};
        for (size_t i{}; i<cloud.size(); ++i)
        {
            if (cloud.points[i].y < z_base.data)
            {
                object_cloud.points[j].x = cloud.points[i].x;
                object_cloud.points[j].y = cloud.points[i].y;
                object_cloud.points[j].z = cloud.points[i].z;
                ++j;
                // std::cout << "Point added" << std::endl;
            }
        }

        size_t no_of_points {};
        std_msgs::Float32MultiArray centroid;
        float x_sum {};
        float y_sum {};
        float z_sum {};

        for (size_t j{}; j<object_cloud.size(); ++j)
        {
            if ((object_cloud.points[j].x != 0) && (object_cloud.points[j].y != 0) && (object_cloud.points[j].z != 0))
            {
                x_sum += object_cloud.points[j].x;
                y_sum += object_cloud.points[j].y;
                z_sum += object_cloud.points[j].z;
                ++no_of_points;
            }
        }

        centroid.data.clear();
        centroid.data.push_back(x_sum);
        centroid.data.push_back(y_sum);
        centroid.data.push_back(z_sum);
        centroid_pub.publish(centroid);

        // Testing to see if the point cloud was filled up.
        // for (size_t j{}; j<object_cloud.size(); ++j)
        // {
        //    if ((object_cloud.points[j].x != 0) && (object_cloud.points[j].y != 0) && (object_cloud.points[j].z != 0))
        //    {
        //        std::cout<< j<<" ";
        //        std::cout<< object_cloud.points[j].x<<" ";
        //        std::cout<< object_cloud.points[j].y<<" ";
        //        std::cout<< object_cloud.points[j].z << std::endl;
        //    }
        // }

        pcl::toROSMsg(object_cloud, object_output);
        obj_pub.publish(object_output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_obj_sub, base_sub;
    ros::Publisher obj_pub, centroid_pub;
    std_msgs::Float32 z_base;
};

main(int argc, char **argv) 
{
    ros::init(argc, argv, "pcl_franka_cube");
    cloudHandler handler;
    ros::spin();
    return 0;
}
