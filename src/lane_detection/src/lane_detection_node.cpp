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

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
