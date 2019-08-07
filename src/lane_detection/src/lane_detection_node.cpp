//
// Created by mechlab on 06.08.19.
//

#include "ros/ros.h"
#include "lane_detector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_detector");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(5);
    htwk::lane_detector detector(node_handle);

    auto kittie_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("kitti", 1);
    pcl::PCLPointCloud2 cloud;
    pcl::PCDReader reader;
    reader.read("/home/mechlab/hackathon/data/pcl", cloud);
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl_conversions::fromPCL(cloud, output_cloud_msg);
    output_cloud_msg.header.frame_id = "map";

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        kittie_publisher.publish(output_cloud_msg);
    }

    return EXIT_SUCCESS;
}
