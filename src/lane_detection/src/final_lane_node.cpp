//
// Created by Mariia,Karl,Friedrich,Nick on 08.08.19.
//

#include "ros/ros.h"
#include "final_lane.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "final_lane");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(5);
    htwk::final_lane detector(node_handle);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
