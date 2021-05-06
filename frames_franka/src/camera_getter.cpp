#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_getter");
    ros::NodeHandle n;
    ros::ServiceClient camera_getter_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    std_msgs::Header header;

    gazebo_msgs::GetModelState get_camera_loc;
    get_camera_loc.request.model_name = "kinect_ros";
    get_camera_loc.request.relative_entity_name = "world";
    bool get_success{};

    geometry_msgs::Pose camera_pose;
    geometry_msgs::Twist camera_twist;

    if (camera_getter_client.call(get_camera_loc))
    {
    camera_pose = get_camera_loc.response.pose;
    camera_twist = get_camera_loc.response.twist;
    header = get_camera_loc.response.header;
    get_success = get_camera_loc.response.success;

    if(get_success = true)
    {
        // printf("\nYes, the pose of kinect was obtained.\n");
        // ROS_INFO("World -> Camera is: [x y z r p y] :");
        // std::cout<<camera_pose.position.x<<std::endl;
        // std::cout<<camera_pose.position.y<<std::endl;
        // std::cout<<camera_pose.position.z<<std::endl;
        // std::cout<<camera_pose.orientation.x<<std::endl;
        // std::cout<<camera_pose.orientation.y<<std::endl;
        // std::cout<<camera_pose.orientation.z<<std::endl;
        // std::cout<<camera_pose.orientation.w<<std::endl;
        // std::cout<<"Header: "<<header.frame_id<<std::endl;
        // ros::Duration(2.0).sleep();
    }
    else
    {
    ROS_ERROR("Failed to call service");
    return 1;
    }
    }
    
   transformStamped.header.frame_id = "world";
   transformStamped.child_frame_id = "camera_link";

  //  transformStamped.transform.translation.x = camera_pose.position.x;
  //  transformStamped.transform.translation.y = camera_pose.position.y;
  //  transformStamped.transform.translation.z = camera_pose.position.z;

   transformStamped.transform.translation.x = 0.6;
   transformStamped.transform.translation.y = 0.75;
   transformStamped.transform.translation.z = 0.85; //Should be actually set by using gazebo_msgs/get_model_state as shown in previous comment

   tf2::Quaternion q;
  //  q.setRPY(camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z);
   q.setRPY(0, 0, 0);
   transformStamped.transform.rotation.x = q.x();
   transformStamped.transform.rotation.y = q.y();
   transformStamped.transform.rotation.z = q.z();
   transformStamped.transform.rotation.w = q.w();

   ros::Rate rate(10.0);
   while (n.ok())
   {
    transformStamped.header.stamp = ros::Time(0);
    tfb.sendTransform(transformStamped);
    rate.sleep();
    // printf("sending\n");
  }
}