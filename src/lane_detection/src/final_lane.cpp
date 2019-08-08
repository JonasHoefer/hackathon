//
// Created by Mariia,Karl,Friedrich,Nick on 08.08.19.
//

#include "final_lane.h"
#include <nav_msgs/Path.h>
#define FRAME_ID "ouster"

htwk::final_lane::final_lane(ros::NodeHandle &handle) noexcept {
    //m_velodyne_points_subscriber = handle.subscribe("/vlp_102/velodyne_points", 1, &htwk::lane_detector::raw_data_callback, this);
    lane_detector_subscriber = handle.subscribe("cloud_seg/lane", 1, &htwk::final_lane::raw_data_callback, this);
    path_publisher = handle.advertise<nav_msgs::Path>("PATH", 1);
}

void htwk::final_lane::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    sensor_msgs::PointCloud2 cloud_msg_transformed;
    pcl_ros::transformPointCloud("ouster", *cloud_msg, cloud_msg_transformed, m_transform);

    pcl::PCLPointCloud2 input_cloud;
    pcl_conversions::toPCL(cloud_msg_transformed, input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

    publish_lane_path(*input_cloud_ptr);
}

void htwk::final_lane::publish_lane_path(const pcl::PointCloud<pcl::PointXYZI> &cloud) noexcept {

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;

    for(int i=0; i< cloud.size(); i++){
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = FRAME_ID;
        pose.pose.position.x =cloud.points.at(i).x;
        pose.pose.position.y =cloud.points.at(i).y;
        pose.pose.position.z =cloud.points.at(i).z;
        pose.pose.orientation.x=pose.pose.orientation.y=pose.pose.orientation.z= 0.0;
        pose.pose.orientation.w=1.0;
        path.poses.push_back(pose);
    }
    path.header.frame_id = FRAME_ID;
    path.header.stamp = ros::Time::now();
    path_publisher.publish(path);
}