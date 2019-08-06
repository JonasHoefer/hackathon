//
// Created by ecke on 2/11/19.
//


#include "pcl_ros/point_cloud.h"
#include "lidar_segmentation.h"


int main(int argc, char **argv) {

    ///* GLOBAL SETTINGS  *********************************************************
    ros::init(argc, argv, "lidar_segmentation");
    ros::NodeHandle nh;
    segmentation seg = segmentation(nh);

    ros::Rate loop_rate(5);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    ///*  GENERAL SETTINGS ********************************************************
//    ros::spin(); // keep spinning only use callbacks

    return EXIT_SUCCESS;
}

