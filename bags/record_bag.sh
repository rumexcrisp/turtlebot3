#!/bin/bash
# This script will enable the user to record a bag file with topics
# @author Julian Baisch julian.baisch@de.kaercher.com
# @copyright Alfred Kaercher SE & Co. KG 2022

function usage {
    echo "Usage:"
    echo "  record_bag.sh -h"
    echo "  record_bag-sh [-b <bag-file-name>]"
    echo ""
    echo "Options:"
    echo "  -b    <bag-file-name>  Name of the recorded bag file"
}

# set default parameter values
bag_name=""
topics=()

# topics+=("/camera/depth/color/points")
topics+=("/camera/depth/color/points_decimate")
# topics+=("/cmd_vel")
# topics+=("/cmd_vel_rc100")
# topics+=("/fisheye_camera/image_transformed/compressed")
# topics+=("/free_cells_vis_array")
# topics+=("/imu")
# topics+=("/initialpose")
# topics+=("/joint_states")
# topics+=("/magnetic_field")
topics+=("/map")
# topics+=("/map_metadata")
# topics+=("/map_updates")
# topics+=("/occupied_cells_vis_array")
# topics+=("/octomap_binary")
# topics+=("/octomap_full")
# topics+=("/odom")
topics+=("/radar_filtered_map")
topics+=("/radar_filtered_point_cloud_centers")
# topics+=("/radar_laser_pcl")
topics+=("/radar_mapping_SOR_filter_out")
topics+=("/radar_raw_map")
topics+=("/radar_raw_point_cloud_centers")
# topics+=("/radar_x_filt_out")
# topics+=("/radar_xy_filt_out")
# topics+=("/radar_xyz_filt_out")
topics+=("/radar_xyzi_filt_out")
topics+=("/realsense_filtered_map")
topics+=("/realsense_filtered_point_cloud_centers")
topics+=("/realsense_mapping_SOR_filter_out")
topics+=("/realsense_raw_map")
topics+=("/realsense_raw_point_cloud_centers")
topics+=("/rosout")
topics+=("/rosout_agg")
topics+=("/scan")
# topics+=("/scan_pcl")
topics+=("/tf")
topics+=("/tf_static")
topics+=("/ti_mmwave/radar_scan_pcl")
topics+=("/trajectory")

# check if paramters were passes
if [ $# -eq 0 ]
  then
    usage
    exit 1
fi

# parse command line options using getopts
while getopts ":b:" opt; do
    case "$opt" in
    # bag file name
    b)
        bag_name="$OPTARG"
        ;;
    # wrong argument, display help message
    \?)
        usage
        exit 1
        ;;
    # No argument given for valid option
    :) 
        echo -e "\e[31mError: Option -$OPTARG requires a name for the bag file\e[0m" >&2
        exit 1
        ;;  
    esac
done

if [ -n "$bag_name" ]; then
    # record the bag file with the supplied name and topics and lz4 compression
    rosbag record --lz4 -O "$bag_name" "${topics[@]}"
else
    echo -e  "\e[31mError: Please supply a bag file name with the -b flag\e[0m"
    exit 1
fi

