#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// TODO:
// 1. Take the ROS point cloud and convert to pcl [DONE]
// 2. Get depth image from the pcl
// 3. Publish the depth image 
// 4. Receive the image and convert to numpy array and realign(in another node)

class PointCloudToImage
{
public:

  //Functions used to convert a pcl Point cloud to convert to a depth image. 
  
  //Sub function use to Project the point cloud to a plane (unorganized cloud).
  pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f origin, Eigen::Vector3f axis_x, Eigen::Vector3f axis_y)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*cloud, *aux_cloud);

    auto normal = axis_x.cross(axis_y);
    Eigen::Hyperplane<float, 3> plane(normal, origin);

    for (auto itPoint = aux_cloud->begin(); itPoint != aux_cloud->end(); itPoint++)
    {
        // project point to plane
        auto proj = plane.projection(itPoint->getVector3fMap());
        itPoint->getVector3fMap() = proj;
    }
return aux_cloud;
}

//Receives the subscriber callback as Pointcloud2, converts it into pcl cloud and sends to the cloud->image conversion function, gets the image and publishes it. 
  void cloud_cb (const sensor_msgs::PointCloud2& cloud)
  {
    if ((cloud.width * cloud.height) == 0)
      {
        ROS_INFO("Cloud not dense!");
        return; //return if the cloud is not dense!
      }
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;      //Creating input cloud to copy the ROS PCL Into
    pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud;        //Creating the cloud into which the plane will be projected
    sensor_msgs::PointCloud2 aux_cloud_ros;               //Creating the ROS Msg which will carry the plane point cloud. 

    pcl::fromROSMsg(cloud, *input_cloud);
    Eigen::Vector3f origin;                               //Need to check the axes and origin again. 
    origin << 0, 0, 0;
    Eigen::Vector3f axis_x;
    axis_x << 0, 0, 1;
    Eigen::Vector3f axis_y;
    axis_y << 0, 1, 0;
    aux_cloud = ProjectToPlane(input_cloud, origin, axis_x, axis_y);          
    pcl::toROSMsg(*aux_cloud, aux_cloud_ros);
    ROS_INFO_STREAM(aux_cloud_ros);
    aux_cloud_pub_.publish(aux_cloud_ros);                //Publishing the plane cloud. 

    // image_pub_.publish (image_); //publish our cloud image
  }

  // Constructor, runs the subscribers and publishers
  PointCloudToImage () : cloud_topic_("pcl_object"),image_topic_("depth_img_object"), aux_cloud_topic_("aux_cloud")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);
    aux_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (aux_cloud_topic_, 1);
  }

private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  std::string aux_cloud_topic_; //aux_cloud
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
  ros::Publisher aux_cloud_pub_; //aux_cloud publisher
  pcl::PointCloud<pcl::PointXYZ> aux_cloud;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}