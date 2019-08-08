//
// Created by Jonas,Mariia,Friedrich,Nick,Karl on 06.08.19.
//
#include "lane_detector.h"

#define FRAME_ID "ouster"


htwk::lane_detector::lane_detector(ros::NodeHandle &handle) noexcept {
    // for 16 level lidar //  m_velodyne_points_subscriber = handle.subscribe("/vlp_102/velodyne_points", 1, &htwk::lane_detector::raw_data_callback, this);
    m_velodyne_points_subscriber = handle.subscribe("/points_raw", 1, &htwk::lane_detector::raw_data_callback, this);
    m_lane_point_publisher = handle.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
}


/*
 * callback processes raw pointcloud data from lidar to small amount of points and publishes them
 * first: transforms the raw data from diagonal to a horizontal level
 * second: apply a intensity filter to get pointcloud with points at a high intensity
 * third: apply a heightfilter to only look at a certain amount of vertical levels
 * fourth: build clusters with euclideanclusterextraction
 * fifth: take the cluster with maximum amoint of points to extract outer lane
 * sixth: divide maximum cluster into 5 areas of pointclouds depending on distance
 * seventh: take 5 average points from that areas
 * eighth: add a Car Offset to the points to move them in the right lane for driving
 */
void htwk::lane_detector::raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept {
    try {
        //transform raw pointcloud to pointcloud to remove diagonal angle
        sensor_msgs::PointCloud2 cloud_msg_transformed;
        pcl_ros::transformPointCloud("odom", *cloud_msg, cloud_msg_transformed, m_transform);

        pcl::PCLPointCloud2 input_cloud;
        pcl_conversions::toPCL(cloud_msg_transformed, input_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(input_cloud, *input_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr = height_filter(intensity_filter(input_cloud_ptr, 6.0),
                                                                              -0.5,
                                                                              0.2);
        if (output_cloud_ptr->empty())
            return;

        //acceleration datastructure KdTree for EuclideanClusterExtraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(output_cloud_ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extraction;
        cluster_extraction.setClusterTolerance(0.7);
        cluster_extraction.setSearchMethod(tree);
        cluster_extraction.setMinClusterSize(20);
        cluster_extraction.setMaxClusterSize(std::numeric_limits<int>::max());
        cluster_extraction.setInputCloud(output_cloud_ptr);

        std::vector<pcl::PointIndices> cluster_indices;
        cluster_extraction.extract(cluster_indices);

        if (cluster_indices.size() == 0)
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

        pcl::PointCloud<pcl::PointXYZI> final_cloud = divideIntoFivePoints(max_cluster_points);


        //building polynom
        tk::spline polynom_with_final_points;
        std::vector<double> pts_x, pts_y;

        for (int i = 0; i < final_cloud.points.size(); i++) {

            pts_x.push_back(final_cloud.points.at(i).x);
            pts_y.push_back(final_cloud.points.at(i).y);
        }
        if (pts_x.size() < 3)
            return;

        polynom_with_final_points.set_points(pts_x, pts_y, false);
        pcl::PointXYZI current_point;
        pcl::PointCloud<pcl::PointXYZI> cloud_from_polynom;
        double offset;
        double m, dx, dy;
        double y1 = polynom_with_final_points(2.0);
        (y1 > 0.0) ? (offset = -4.5) : (offset = 2.5);

        for (int i = 0; i < 5; i++) {

            current_point.x = (i + 1.5) * 2.0;
            current_point.y = polynom_with_final_points((i + 1.5) * 2.0);
            // Anstieg am Punkt
            m = polynom_with_final_points.deriv(1, current_point.x);
            //x- and y- offset components , atan(m) = alpha
            dy = offset * cos(atan(m));
            dx = offset * sin(atan(m));
            current_point.x-= dx;
            current_point.y+= dy;
            current_point.z = 0.0;
            current_point.intensity = 10.0;
            cloud_from_polynom.points.push_back(current_point);
        }

        pcl::PCLPointCloud2 output_cloud;
        pcl::toPCLPointCloud2(cloud_from_polynom, output_cloud);
        publish_lane(output_cloud);
    } catch (const std::exception &e) {

    }

}

void htwk::lane_detector::publish_lane(const pcl::PCLPointCloud2 &cloud) noexcept {
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl_conversions::fromPCL(cloud, output_cloud_msg);
    output_cloud_msg.header.frame_id = FRAME_ID;
    m_lane_point_publisher.publish(output_cloud_msg);
}


pcl::PointCloud<pcl::PointXYZI>::Ptr
htwk::lane_detector::intensity_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, float minimum) noexcept {
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> intensity_filter;
    intensity_filter.setFilterFieldName("intensity");
    intensity_filter.setFilterLimits(minimum, FLT_MAX);
    intensity_filter.setInputCloud(input);
    intensity_filter.filter(*intensity_filtered_points);

    return intensity_filtered_points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
htwk::lane_detector::height_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, float min, float max) noexcept {
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> height_filter;
    height_filter.setFilterFieldName("z");
    height_filter.setFilterLimits(min, max);
    height_filter.setInputCloud(input);
    height_filter.filter(*height_filtered_points);

    return height_filtered_points;
}

pcl::PointXYZI htwk::lane_detector::average_point(pcl::PointCloud<pcl::PointXYZI> cluster_points) {
    pcl::PointXYZI wayPoint;

    for (int i = 0; i < cluster_points.size(); i++) {
        wayPoint.x = wayPoint.x + cluster_points.points[i].x;
        wayPoint.y = wayPoint.y + cluster_points.points[i].y;
        wayPoint.z = wayPoint.z + cluster_points.points[i].z;
    }
    wayPoint.x = wayPoint.x / cluster_points.size() * 1.0;
    wayPoint.y = wayPoint.y / cluster_points.size() * 1.0;
    wayPoint.z = wayPoint.z / cluster_points.size() * 1.0;
    wayPoint.intensity = 10.0;

    return wayPoint;
}

/* static Offset
pcl::PointCloud<pcl::PointXYZI>
htwk::lane_detector::setCarOffset(pcl::PointCloud<pcl::PointXYZI> after_reducing_to_5points_cloud) noexcept {
    float offset;
    (after_reducing_to_5points_cloud.points.at(0).y > 0.0) ? (offset = -4.5) : (offset = 2.5);
    for (int i = 0; i < after_reducing_to_5points_cloud.points.size(); i++) {
        after_reducing_to_5points_cloud.points.at(i).y += offset;
    }
    return after_reducing_to_5points_cloud;
}
*/
pcl::PointCloud<pcl::PointXYZI>
htwk::lane_detector::divideIntoFivePoints(pcl::PointCloud<pcl::PointXYZI> max_cluster_point_cloud) noexcept {
    std::vector<int> distanceVector;
    pcl::PointCloud<pcl::PointXYZI> wp1, wp2, wp3, wp4, wp5;

    for (int i = 0; i < max_cluster_point_cloud.size(); i++) {
        float distance = std::sqrt(max_cluster_point_cloud.at(i).x * max_cluster_point_cloud.at(i).x +
                                   max_cluster_point_cloud.at(i).y * max_cluster_point_cloud.at(i).y +
                                   max_cluster_point_cloud.at(i).z * max_cluster_point_cloud.at(i).z);

        if (distance < 6.0) {
            wp1.points.push_back(max_cluster_point_cloud.at(i));
        } else if (distance < 7.0) {
            wp2.points.push_back(max_cluster_point_cloud.at(i));
        } else if (distance < 8.0) {
            wp3.points.push_back(max_cluster_point_cloud.at(i));
        } else if (distance < 9.0) {
            wp4.points.push_back(max_cluster_point_cloud.at(i));
        } else {
            wp5.points.push_back(max_cluster_point_cloud.at(i));
        }

    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_list;
    cloud_list.push_back(wp1);
    cloud_list.push_back(wp2);
    cloud_list.push_back(wp3);
    cloud_list.push_back(wp4);
    cloud_list.push_back(wp5);


    pcl::PointCloud<pcl::PointXYZI> cloud_with_five_points;

    for (int i = 0; i < cloud_list.size(); i++) {
        if (cloud_list.at(i).size() != 0)
            cloud_with_five_points.points.push_back(average_point(cloud_list.at(i)));
    }

    return cloud_with_five_points;
}
/*
pcl::PointCloud<pcl::PointXYZI> htwk::lane_detector::buildMaxOfEuclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr prefilteredCloudPtr) noexcept{

//acceleration datastructure KdTree for EuclideanClusterExtraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(prefilteredCloudPtr);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extraction;
    cluster_extraction.setClusterTolerance(0.7);
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setMinClusterSize(20);
    cluster_extraction.setMaxClusterSize(std::numeric_limits<int>::max());
    cluster_extraction.setInputCloud(prefilteredCloudPtr);

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extraction.extract(cluster_indices);

    if(cluster_indices.size() ==0)
        throw;
    pcl::PointIndices max_cluster_indices = *std::max_element(cluster_indices.begin(), cluster_indices.end(),
                                                              [](const pcl::PointIndices &a,
                                                                 const pcl::PointIndices &b) {
                                                                  return a.indices.size() < b.indices.size();
                                                              });
    pcl::PointCloud<pcl::PointXYZI> max_cluster_points;
    for (int index : max_cluster_indices.indices) {
        max_cluster_points.points.push_back((*prefilteredCloudPtr)[index]);
    }

    return max_cluster_points;
}
*/