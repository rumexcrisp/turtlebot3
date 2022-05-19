#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/fusionConfig.h>


#ifndef SENSOR_FUSION_HPP_
#define SENSOR_FUSION_HPP_


class SensorFusion
{
public:
    SensorFusion(ros::NodeHandle _nh);
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void RealsenseCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void ReconfigureCallback(sensor_fusion::fusionConfig &config, uint32_t level);

private:
    ros::NodeHandle m_nh;
    laser_geometry::LaserProjection m_projector;
    tf::TransformListener m_tfListener;
    // tf2_ros::TransformListener m_tf2Listener;

    ros::Publisher m_pcl_publisher;
    ros::Publisher m_realsense_publisher;
    ros::Subscriber m_laser_subscriber;
    ros::Subscriber m_realsense_subscriber;

    double m_decimation_x;
    double m_decimation_y;
    double m_decimation_z;
};

#endif