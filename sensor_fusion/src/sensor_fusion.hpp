#include <ros/ros.h>
#include <tf/transform_listener.h>
// #include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#ifndef SENSOR_FUSION_HPP_
#define SENSOR_FUSION_HPP_


class SensorFusion
{
public:
    SensorFusion();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void RealsenseCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

private:
    ros::NodeHandle m_nh;
    laser_geometry::LaserProjection m_projector;
    tf::TransformListener m_tfListener;
    // tf2_ros::TransformListener m_tf2Listener;

    ros::Publisher m_pcl_publisher;
    ros::Publisher m_realsense_publisher;
    ros::Subscriber m_laser_subscriber;
    ros::Subscriber m_realsense_subscriber;
};

#endif