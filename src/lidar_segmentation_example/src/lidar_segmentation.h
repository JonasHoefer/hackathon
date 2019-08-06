//
// Created by ecke on 2/11/19.
//

#ifndef PROJECT_SEGMENTATION_H
#define PROJECT_SEGMENTATION_H
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
#include <string>
//#include <obj_recognition/SegmentedClustersArray.h>
//#include <obj_recognition/ClusterData.h>


class segmentation {
public:

    segmentation(ros::NodeHandle);

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    ros::Publisher m_clusterPub, m_markerArray, m_Ground,pcl_seg, pcl_ground, pcl_objects, pcl_lane;
    
    void extract_objects(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    void extract_surface(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};



#endif //PROJECT_SEGMENTATION_H
