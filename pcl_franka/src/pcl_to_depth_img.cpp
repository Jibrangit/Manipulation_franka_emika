#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <string>

class PointCloudToImage
{
public:
  void
  cloud_cb (const sensor_msgs::PointCloud2& cloud)
  {
    if ((cloud.width * cloud.height) == 0)
      {
        ROS_INFO("Cloud not dense!");
        return; //return if the cloud is not dense!
      }
      
    try
    {
      pcl::fromROSMsg(cloud, color_cloud);

      for(size_t i = 0; i<color_cloud.size(); i++)
      {
        color_cloud[i].r = 0;
        color_cloud[i].g = 0;
        color_cloud[i].b = 0;
        color_cloud[i].a = 0;
      }
      std::cout<<"Height and width of point cloud: "<<cloud.height<<" "<<cloud.width<<std::endl;
      pcl::toROSMsg (color_cloud, image_); //convert the cloud
      std::cout<<"Sample points from depth image: "<<image_.data[0]<<" "<<image_.data[100]<<std::endl;
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub_.publish (image_); //publish our cloud image
  }
  PointCloudToImage () : cloud_topic_("pcl_object"),image_topic_("depth_img_object")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  pcl::PointCloud<pcl::PointXYZRGBA> color_cloud;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}