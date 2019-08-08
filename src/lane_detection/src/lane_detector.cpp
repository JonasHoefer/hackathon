//
// Created by mechlab on 06.08.19.
//

#include "lane_detector.h"
#include <cmath>
#include <vector>
#define FRAME_ID "ouster"


pcl::PointXYZI average_point(pcl::PointCloud<pcl::PointXYZI> cluster_points);


htwk::lane_detector::lane_detector(ros::NodeHandle &handle) noexcept {
    //m_velodyne_points_subscriber = handle.subscribe("/vlp_102/velodyne_points", 1, &htwk::lane_detector::raw_data_callback, this);
    m_velodyne_points_subscriber = handle.subscribe("/points_raw", 1, &htwk::lane_detector::raw_data_callback, this);
    m_lane_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
}

void htwk::lane_detector::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    try{
        sensor_msgs::PointCloud2 cloud_msg_transformed;
        pcl_ros::transformPointCloud("odom", *cloud_msg, cloud_msg_transformed, m_transform);

        pcl::PCLPointCloud2 input_cloud;
        pcl_conversions::toPCL(cloud_msg_transformed, input_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr = height_filter(intensity_filter(input_cloud_ptr, 6.0 ), -0.5,
                                                                              0.2);

        if (output_cloud_ptr->empty())
            return;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(output_cloud_ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extraction;
        cluster_extraction.setClusterTolerance(0.5);
        cluster_extraction.setSearchMethod(tree);
        cluster_extraction.setMinClusterSize(4);
        cluster_extraction.setMaxClusterSize(std::numeric_limits<int>::max());
        cluster_extraction.setInputCloud(output_cloud_ptr);

        std::vector<pcl::PointIndices> cluster_indices;
        cluster_extraction.extract(cluster_indices);

        if(cluster_indices.size() == 0)
            return;
        pcl::PointIndices max_cluster_indices = *std::max_element(cluster_indices.begin(), cluster_indices.end(),
                                                                  [](const pcl::PointIndices &a,
                                                                     const pcl::PointIndices &b) {
                                                                      return a.indices.size() < b.indices.size();
                                                                  });
        pcl::PointCloud<pcl::PointXYZI> max_cluster_points;
        for (int index : max_cluster_indices.indices) {
            max_cluster_points.points.push_back((*output_cloud_ptr)[index]);
        }

        //new cloud for points
        std::vector<int> distanceVector;
        pcl::PointCloud<pcl::PointXYZI> wp1, wp2, wp3, wp4, wp5;

        //Get Points from Cloud
        std::cout << "FASHKASHFKHKASFHJHASFHASHF" << std::endl;
        for (int i = 0; i < max_cluster_points.size(); i++) {
            float distance = std::sqrt(max_cluster_points.at(i).x * max_cluster_points.at(i).x +
                                       max_cluster_points.at(i).y * max_cluster_points.at(i).y +
                                       max_cluster_points.at(i).z * max_cluster_points.at(i).z);
            /* std::cout << distance << std::endl; */

            if (distance < 4.4) {
                wp1.points.push_back(max_cluster_points.at(i));
            } else if (distance < 5.8) {
                wp2.points.push_back(max_cluster_points.at(i));
            } else if (distance < 7.2) {
                wp3.points.push_back(max_cluster_points.at(i));
            } else if (distance < 8.6) {
                wp4.points.push_back(max_cluster_points.at(i));
            } else {
                wp5.points.push_back(max_cluster_points.at(i));
            }

        }

        std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_list;
        cloud_list.push_back(wp1);
        cloud_list.push_back(wp2);
        cloud_list.push_back(wp3);
        cloud_list.push_back(wp4);
        cloud_list.push_back(wp5);


       /* WayPointBMW1.x = 0.0;
        WayPointBMW1.y = 0.0;
        WayPointBMW1.intensity = 2.0; */
        pcl::PointCloud<pcl::PointXYZI> final_output_cloud;

        for (int i=0; i < cloud_list.size(); i++){
            if(cloud_list.at(i).size() != 0)
                final_output_cloud.points.push_back(average_point(cloud_list.at(i)));
        }

        //setOffset
        float offset;

        (final_output_cloud.points.at(0).y > 0.0)? (offset = -5.4): (offset = 2.5);
        for (int i = 0; i < final_output_cloud.points.size(); i++){
            final_output_cloud.points.at(i).y += offset;
        }

        std::cout << "WAYPOINTS" << std::endl;
        for(int i=0; i < final_output_cloud.size(); i++){
            std::cout << final_output_cloud.points.at(i) << std:: endl;
        }



        pcl::PCLPointCloud2 output_cloud;
        pcl::toPCLPointCloud2(final_output_cloud, output_cloud);

        double time_end = ros::Time::now().toSec() + (double)ros::Time::now().toNSec()/1000000000.0;

        //ROS_INFO_STREAM("diff time: " << time_end - time_beg);

        publish_lane(output_cloud);
    } catch(const std::exception& e){
        std::cout << "EXCEPTION" << std::endl;
    }

}

void htwk::lane_detector::publish_lane(const pcl::PCLPointCloud2 &cloud) noexcept {
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl_conversions::fromPCL(cloud, output_cloud_msg);
    output_cloud_msg.header.frame_id = FRAME_ID;
    m_lane_point_publisher.publish(output_cloud_msg);
}



pcl::PointCloud<pcl::PointXYZI>::Ptr htwk::lane_detector::intensity_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, float minimum) noexcept {
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> intensity_filter;
    intensity_filter.setFilterFieldName("intensity");
    intensity_filter.setFilterLimits(minimum, FLT_MAX);
    intensity_filter.setInputCloud(input);
    intensity_filter.filter(*intensity_filtered_points);

    return intensity_filtered_points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr htwk::lane_detector::height_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, float min, float max) noexcept {
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> height_filter;
    height_filter.setFilterFieldName("z");
    height_filter.setFilterLimits(min, max);
    height_filter.setInputCloud(input);
    height_filter.filter(*height_filtered_points);

    return height_filtered_points;
}

pcl::PointXYZI average_point(pcl::PointCloud<pcl::PointXYZI> cluster_points){
    pcl::PointXYZI wayPoint ;

    for (int i = 0; i < cluster_points.size(); i++) {
        wayPoint.x = wayPoint.x + cluster_points.points[i].x;
        wayPoint.y = wayPoint.y + cluster_points.points[i].y;
        wayPoint.z = wayPoint.z + cluster_points.points[i].z;
    }
    wayPoint.x = wayPoint.x / cluster_points.size() *1.0  ;
    wayPoint.y = wayPoint.y / cluster_points.size() *1.0;
    wayPoint.z = wayPoint.z / cluster_points.size() *1.0;
    wayPoint.intensity = 10.0;

    return wayPoint;
}



