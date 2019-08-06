//
// Created by mechlab on 06.08.19.
//

#include "lane_detector.h"

#define FRAME_ID "odom"


htwk::lane_detector::lane_detector(ros::NodeHandle &handle) noexcept {
    m_velodyne_points_subscriber = handle.subscribe("/vlp_102/velodyne_points", 1, &htwk::lane_detector::raw_data_callback, this);
    m_lane_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
}

void htwk::lane_detector::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    pcl::PCLPointCloud2 input_cloud;
    pcl_conversions::toPCL(*cloud_msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> intensity_filter;
    intensity_filter.setFilterFieldName("intensity");
    intensity_filter.setFilterLimits(100, FLT_MAX);
    intensity_filter.setInputCloud(input_cloud_ptr);
    intensity_filter.filter(*filtered_points);

    pcl::PCLPointCloud2 output_cloud;
    pcl::toPCLPointCloud2(*filtered_points, output_cloud);

    publish_lane(output_cloud);
}

void htwk::lane_detector::publish_lane(const pcl::PCLPointCloud2 &cloud) {
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl_conversions::fromPCL(cloud, output_cloud_msg);
    output_cloud_msg.header.frame_id = FRAME_ID;
    m_lane_point_publisher.publish(output_cloud_msg);
}
