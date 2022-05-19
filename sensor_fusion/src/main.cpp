// #include <ros/ros.h>
#include "sensor_fusion.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion");
    
    ros::NodeHandle nh("~");
    
    SensorFusion fusion(nh);
    
    ros::spin();
    
    return 0;
}