//
// Created by mechlab on 06.08.19.
//

#include "lane_detector.h"

htwk::lane_detector::lane_detector(ros::NodeHandle &handle) noexcept {
    m_velodyne_points_subscriber = handle.subscribe("/vlp_102/velodyne_points", 1, &htwk::lane_detector::raw_data_callback, this);
    m_lane_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
}

void htwk::lane_detector::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    m_lane_point_publisher.publish(cloud_msg);
}
