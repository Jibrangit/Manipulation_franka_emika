#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

class cloudHandler
{
public:
    cloudHandler()
    {
        base_sub = nh.subscribe("base_location", 10, &cloudHandler::BaseLocationCB, this);
        pcl_obj_sub = nh.subscribe("pcl_franka_filtered", 10, &cloudHandler::cloudCB, this);
        obj_pub  = nh.advertise<sensor_msgs::PointCloud2>("pcl_object", 1);
        centroid_pub = nh.advertise<geometry_msgs::Point>("object_centroid", 10);
        // object_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("ObjPointCloud", 1);
    }

    void BaseLocationCB(const std_msgs::Float32 &base_location)
    {
        z_base.data = base_location.data;
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> object_cloud;
        pcl::PointCloud<pcl::PointXYZRGBA> ObjPointCloud;
        sensor_msgs::PointCloud2 object_output;
        sensor_msgs::PointCloud2 obj_point_cloud;


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
        // std_msgs::Float32MultiArray centroid;
        // float x_sum {};
        // float y_sum {};
        // float z_sum {};
        geometry_msgs::Point centroid;
        double x_sum;
        double y_sum;
        double z_sum;

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

        centroid.x = x_sum/no_of_points;
        centroid.y = y_sum/no_of_points;
        centroid.z = z_sum/no_of_points;

        // std::cout<< "\nCamera -> Centroid(pcl)\n";
        // std::cout<< centroid.x<< std::endl;
        // std::cout<< centroid.y<< std::endl;
        // std::cout<< centroid.z<< std::endl;
        centroid_pub.publish(centroid);
        // ros::Duration(1.0).sleep();

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
        // for(auto i=0; i<object_cloud.size(); i++)
        // {
        //     ObjPointCloud[i].x = object_cloud[i].x;
        //     ObjPointCloud[i].y = object_cloud[i].y;
        //     ObjPointCloud[i].z = object_cloud[i].z;
        //     ObjPointCloud[i].r = 127;
        //     ObjPointCloud[i].g = 0;
        //     ObjPointCloud[i].b = 0;
        //     ObjPointCloud[i].a = 0;
        // }
        // pcl::io::savePCDFileASCII("/home/jibran_old/catkin_ws/src/Manipulation_franka_emika/pcl_franka/pcd/object.pcd", ObjPointCloud);

        
        // pcl::toROSMsg(ObjPointCloud, obj_point_cloud);
        // object_point_cloud.publish(obj_point_cloud);
        // std::cout<<"Object cloud has ht, width: "<<object_output.height<<" "<<object_output.width<<std::endl;
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_obj_sub, base_sub;
    ros::Publisher obj_pub, centroid_pub, object_point_cloud;
    std_msgs::Float32 z_base;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_franka_cube");
    cloudHandler handler;
    ros::spin();
    return 0;
}