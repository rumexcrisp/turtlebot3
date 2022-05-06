#include "sensor_fusion.hpp"

/**
 * Constructor
 */
SensorFusion::SensorFusion()
{
    m_laser_subscriber = m_nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, &SensorFusion::ScanCallback, this);
    m_realsense_subscriber = m_nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, &SensorFusion::RealsenseCallback, this);
    m_pcl_publisher = m_nh.advertise<sensor_msgs::PointCloud2>("/scan_pcl", 100, false);
    m_realsense_publisher = m_nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_decimate", 100, false);
    m_tfListener.setExtrapolationLimit(ros::Duration(0.1));
    // m_tf2Listener.setExtrapolationLimit(ros::Duration(0.1));
}

void SensorFusion::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (!m_tfListener.waitForTransform(scan->header.frame_id, "base_scan", scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment), ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    m_projector.transformLaserScanToPointCloud("base_scan", *scan, cloud, m_tfListener);
    m_pcl_publisher.publish(cloud);
}

void SensorFusion::RealsenseCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_cloud_filtered(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(*cloud, *pcl_cloud);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*pcl_cloud_filtered);

    m_realsense_publisher.publish(pcl_cloud_filtered);

    return;
}