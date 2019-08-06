//
// Created by mechlab on 06.08.19.
//

#ifndef HACKATHON_LANE_DETECTOR_H
#define HACKATHON_LANE_DETECTOR_H

#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


namespace htwk {
    class lane_detector {
    public:
        explicit lane_detector(ros::NodeHandle &handle) noexcept;

    private:
        tf::TransformListener m_transform;
        ros::Subscriber m_velodyne_points_subscriber;
        ros::Publisher m_lane_point_publisher;

    private:
        void raw_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) noexcept;

        void publish_lane(const pcl::PCLPointCloud2 &cloud) noexcept;

        static pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, float minimum) noexcept;

        static pcl::PointCloud<pcl::PointXYZI>::Ptr height_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, float min, float max) noexcept;
    };
}


#endif //HACKATHON_LANE_DETECTOR_H
