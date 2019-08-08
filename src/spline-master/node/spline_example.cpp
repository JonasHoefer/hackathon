

//
// Created by ecke on 8/7/19.
//
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "spline.h"

int main(int argc, char **argv)
{

    std::string param;
    ros::init(argc, argv, "spline_example_node");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // create and init provider
    double rate = 5.0;
    ros::Publisher pub_points = n.advertise<visualization_msgs::Marker>("/spline_points",1);
    tk::spline s;  // create spline instance
    std::vector<double> pts_x, pts_y;
    for (double i=0; i<5; i++){

        pts_x.push_back(i);
        pts_y.push_back(i+i/4);
    }


    s.set_points(pts_x,pts_y);
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "base_link";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.ns = 'points';

    points.pose.orientation.w = 1.0;
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.color.r = 1.0;
    points.color.a = 1.0; // make them visible

    geometry_msgs::Point p;

    ros::Rate r(rate); //hz
    double x_val[4]=  {1.0, 2.0, 5.0, 10.0};
    while (ros::ok())
    {
        for (auto x:x_val){
            p.x = x;
            p.y = s(x);
            p.z = 0;
            points.points.push_back(p);


        }
        pub_points.publish(points);
        ros::spinOnce();
        r.sleep();
    }

    return EXIT_SUCCESS;
}