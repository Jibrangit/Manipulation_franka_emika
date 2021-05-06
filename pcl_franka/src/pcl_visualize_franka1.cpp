#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler
{
public:
    cloudHandler()
        :viewer("Cloud Viewer")
        {
            output_sub = nh.subscribe("pcl_object", 1, &cloudHandler::outputCB, this);

            viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

            viewer.createViewPort(0.0, 0.0, 0.5, 1.0, output_view);
            viewer.setBackgroundColor(0, 0, 0, output_view);

            viewer.addCoordinateSystem(1.0);
            viewer.initCameraParameters();

        }
    
    void outputCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        viewer.removeAllPointClouds(output_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "output", output_view);
    }

    void timerCB(const ros::TimerEvent&)
    {
        viewer.spinOnce();
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber output_sub;
    pcl::visualization::PCLVisualizer viewer;
    int output_view;
    ros::Timer viewer_timer;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_visualizer");

    cloudHandler handler;

    ros::spin();

    return 0;
}