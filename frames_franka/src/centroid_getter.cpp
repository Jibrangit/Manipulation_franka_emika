#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/Header.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

// class CentroidGetter 
// {
// public:
//   CentroidGetter()
//   {
//     // location_sub = nh.subscribe("object_centroid",10, &CentroidGetter::locationCB, this);
//     // std::cout<<"\nSubscribed";
//   }
//   void locationCB(const std_msgs::Float32MultiArray &location_c)
//   {
//     location = location_c;
//     std::cout<<"z location is "<<location.data.at(2)<<std::endl;
//     ros::Duration(1.0).sleep();
//   }

//   // std_msgs::Float32MultiArray location_getter()
//   // {
//   //   return location;
//   // }

//   std_msgs::Float32MultiArray location;
// // private:
// // ros::NodeHandle nh;
// // ros::Subscriber location_sub;
// };

// int main(int argc, char** argv){
//   ros::init(argc, argv, "object_frame_broadcaster");
//   ros::NodeHandle node;
//   ros::Subscriber obj_sub;

//   tf2_ros::TransformBroadcaster tfb;
//   geometry_msgs::TransformStamped transformStamped;
  
//   CentroidGetter centroid;
//   obj_sub = node.subscribe("object_centroid",10, &CentroidGetter::locationCB, &centroid);

//   // while(centroid.location.data.size() != 3)
//   // {
//   //   CentroidGetter centroid2;
//   //   centroid.location = centroid2.location;
//   //   // std::cout<<"\nGetting centroid location, please wait\n";
//   //   ros::Duration(1.0).sleep();
//   // }

//   transformStamped.header.frame_id = "camera_link";
//   transformStamped.child_frame_id = "object_frame";

//   transformStamped.transform.translation.x = centroid.location.data.at(0);
//   transformStamped.transform.translation.y = centroid.location.data.at(1);
//   transformStamped.transform.translation.z = centroid.location.data.at(2);
//   tf2::Quaternion q;
//         q.setRPY(0, 0, 0);
//   transformStamped.transform.rotation.x = q.x();
//   transformStamped.transform.rotation.y = q.y();
//   transformStamped.transform.rotation.z = q.z();
//   transformStamped.transform.rotation.w = q.w();

//   // ros::Rate rate(10.0);
//   // while (node.ok()){
//   try
//   {
//     transformStamped.header.stamp = ros::Time::now();
//     tfb.sendTransform(transformStamped);
//   }
//   catch(const std::exception& e)
//   {
//     std::cerr << e.what() << '\n';
//     std::cout<<"Object frame not broadcasted.";
//   }
//     // rate.sleep();
//     // printf("sending\n");
//   ros::spin();
// };

class Centroid 
{
public:
    Centroid()
    {
        centroid_sub = nh.subscribe("object_centroid", 10, &Centroid::centroidCB, this);
        transformStamped.header.frame_id = "camera_link";
        transformStamped.child_frame_id = "object_frame";
        while(nh.ok())
        {
            publish_frame();
            ros::spinOnce();
            // ros::Rate r(10.0);
            // r.sleep();
            // printf("COnstructor called.");
        }
    }
    void centroidCB(const geometry_msgs::Point &centroid)
    {
        location = centroid;
        // printf("Callback received");
    }

    void publish_frame()
    {
        transformStamped.transform.translation.x = -location.x;
        transformStamped.transform.translation.y = -location.z;
        transformStamped.transform.translation.z = -location.y;
        tf2::Quaternion q;
                q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        IK_location.x = transformStamped.transform.translation.x;
        IK_location.y = transformStamped.transform.translation.y;
        IK_location.z = transformStamped.transform.translation.z;

        // std::cout<< "Camera -> Object(tf2)"<< std::endl;
        // std::cout<< "\n"<< IK_location.x <<" "<< IK_location.y <<" "<< IK_location.z << std::endl;
        // ros::Duration(1.0).sleep();

        ros::Rate rate(10.0);
        try
        {
          transformStamped.header.stamp = ros::Time(0);
          tfb.sendTransform(transformStamped);
        }
        catch(const std::exception& e)
        {
          std::cerr << e.what() << '\n';
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber centroid_sub;
    geometry_msgs::Point location;
    geometry_msgs::Point IK_location;
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "centroid_getter");
    Centroid Centroid;
    return 0;
}