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

  //Function used to convert a pcl Point cloud to convert to a depth image. 
  cv::Mat makeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2)
      {
        pcl::PointXYZ cloudMin, cloudMax;
        pcl::getMinMax3D(*cloud, cloudMin, cloudMax);
        std::cout<<"Max and min of cloud: "<<cloudMin<<" "<<cloudMax<<std::endl;

        std::string dimen1, dimen2;
        float dimen1Max, dimen1Min, dimen2Min, dimen2Max;
        if (dimensionToRemove == "x")
        {
            dimen1 = "y";
            dimen2 = "z";
            dimen1Min = cloudMin.y;
            dimen1Max = cloudMax.y;
            dimen2Min = cloudMin.z;
            dimen2Max = cloudMax.z;
        }
        else if (dimensionToRemove == "y")
        {
            dimen1 = "x";
            dimen2 = "z";
            dimen1Min = cloudMin.x;
            dimen1Max = cloudMax.x;
            dimen2Min = cloudMin.z;
            dimen2Max = cloudMax.z;
        }
        else if (dimensionToRemove == "z")
        {
            dimen1 = "x";
            dimen2 = "y";
            dimen1Min = cloudMin.x;
            dimen1Max = cloudMax.x;
            dimen2Min = cloudMin.y;
            dimen2Max = cloudMax.y;
        }

        std::vector<std::vector<int>> pointCountGrid;
        int maxPoints = 0;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> grid;

        for (float i = dimen1Min; i < dimen1Max; i += stepSize1)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr slice = passThroughFilter1D(cloud, dimen1, i, i + stepSize1, 1);
            grid.push_back(slice);

            std::vector<int> slicePointCount;

            for (float j = dimen2Min; j < dimen2Max; j += stepSize2)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cell = passThroughFilter1D(slice, dimen2, j, j + stepSize2, 1);

                int gridSize = grid_cell->size();
                slicePointCount.push_back(gridSize);

                if (gridSize > maxPoints)
                {
                    maxPoints = gridSize;
                }
            }
            pointCountGrid.push_back(slicePointCount);
        }

        cv::Mat mat(static_cast<int>(pointCountGrid.size()), static_cast<int>(pointCountGrid.at(0).size()), CV_8UC1);
        mat = cv::Scalar(0);

        for (int i = 0; i < mat.rows; ++i)
        {
            for (int j = 0; j < mat.cols; ++j)
            {
                int pointCount = pointCountGrid.at(i).at(j);
                float percentOfMax = (pointCount + 0.0) / (maxPoints + 0.0);
                int intensity = percentOfMax * 255;

                mat.at<uchar>(i, j) = intensity;
            }
        }

        return mat;
    }

    //Helper function used by the image conversion function for some filtering process, idk. 
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const double low, const double high, const bool remove_inside)
      {
      if (low > high)
      {
          std::cout << "Warning! Min is greater than max!\n";
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;

      pass.setInputCloud(cloud);
      pass.setFilterFieldName(field);
      pass.setFilterLimits(low, high);
      pass.setFilterLimitsNegative(remove_inside);
      pass.filter(*cloud_filtered);
      return cloud_filtered;
    }

//Receives the subscriber callback as Pointcloud2, converts it into pcl cloud and sends to the cloud->image conversion function, gets the image and publishes it. 
  void cloud_cb (const sensor_msgs::PointCloud2& cloud)
  {
    if ((cloud.width * cloud.height) == 0)
      {
        ROS_INFO("Cloud not dense!");
        return; //return if the cloud is not dense!
      }
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud;
    pcl::fromROSMsg(cloud, *depth_cloud);
    cv::Mat depth_img = makeImageFromPointCloud(depth_cloud, "z", 1.0, 1.0);


    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id   = "camera_link"; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = depth_img; // Your cv::Mat

    image_pub_.publish(out_msg.toImageMsg());


  
    image_pub_.publish (image_); //publish our cloud image
  }

  // Constructor, runs the subscribers and publishers
  PointCloudToImage () : cloud_topic_("pcl_object"),image_topic_("depth_img_object")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);
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