//
// Created by mechlab on 06.08.19.
//

#include "lane_detector.h"

#define FRAME_ID "ouster"


htwk::lane_detector::lane_detector(ros::NodeHandle &handle) noexcept {
    m_lidar_points_subscriber = handle.subscribe("/points_raw", 1, &htwk::lane_detector::raw_data_callback, this);
    m_lane_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
    m_trajectory_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("lane/points", 1);
}

void htwk::lane_detector::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    sensor_msgs::PointCloud2 cloud_msg_transformed;
    pcl_ros::transformPointCloud("odom", *cloud_msg, cloud_msg_transformed, m_transform);

    pcl::PCLPointCloud2 input_cloud;
    pcl_conversions::toPCL(cloud_msg_transformed, input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

    // apply filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr = height_filter(intensity_filter(input_cloud_ptr, 5), -0.5, 0.2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr largest_cluster_ptr = extract_largest_cluster(filtered_cloud_ptr);

    if (largest_cluster_ptr->points.empty())
        return;

    // points with in different zones
    float min_distance = std::numeric_limits<float>::max(), max_distance = 0.0f;
    for (auto &point : largest_cluster_ptr->points) {
        point.intensity = point.x * point.x + point.y * point.y + point.z * point.z;
        min_distance = point.intensity < min_distance ? point.intensity : min_distance;
        max_distance = point.intensity > max_distance ? point.intensity : max_distance;
    }

    float zone_radius = 18.0f; // (max_distance - min_distance) / ZONE_COUNT;
    pcl::PointCloud<pcl::PointXYZ> output;
    for (int i = 0; i < max_distance - min_distance; ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr zone(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> intensity_filter;
        intensity_filter.setFilterFieldName("intensity");
        intensity_filter.setFilterLimits(min_distance + (i * zone_radius), min_distance + ((i + 1) * zone_radius));
        intensity_filter.setInputCloud(largest_cluster_ptr);
        intensity_filter.filter(*zone);

        if (zone->points.empty())
            continue;

        Eigen::Matrix<float, 4, 1> point;
        pcl::compute3DCentroid(*zone, point);
        output.push_back(pcl::PointXYZ(point[0], point[1] - 2.5, point[2]));
    }

    // output
    pcl::PCLPointCloud2 output_cloud;
    pcl::toPCLPointCloud2(output, output_cloud);
    publish_lane(output_cloud);
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

pcl::PointCloud<pcl::PointXYZI>::Ptr htwk::lane_detector::extract_largest_cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input) noexcept {
    pcl::PointCloud<pcl::PointXYZI>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZI>);

    if (!input->empty()) {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(input);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extraction;
        cluster_extraction.setClusterTolerance(0.5);
        cluster_extraction.setSearchMethod(tree);
        cluster_extraction.setMinClusterSize(10);
        cluster_extraction.setMaxClusterSize(std::numeric_limits<int>::max());
        cluster_extraction.setInputCloud(input);

        std::vector<pcl::PointIndices> cluster_indices;
        cluster_extraction.extract(cluster_indices);

        if (cluster_indices.empty())
            return largest_cluster;

        pcl::PointIndices max_cluster_indices = *std::max_element(cluster_indices.begin(), cluster_indices.end(),
                                                                  [](const pcl::PointIndices &a, const pcl::PointIndices &b) { return a.indices.size() < b.indices.size(); });

        for (int index : max_cluster_indices.indices) {
            largest_cluster->points.push_back((*input)[index]);
        }
    }

    return largest_cluster;
}
