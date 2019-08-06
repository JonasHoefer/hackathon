//
// Created by ecke on 2/11/19.
//

#include "lidar_segmentation.h"
#include <chrono>  // for high_resolution_clock

#define FRAME_ID "odom"

// Callback for cloud segmentation example  
void segmentation::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Record start time
    auto start = std::chrono::high_resolution_clock::now();

    // convert from sensor_msgs to pcl PCLPointCloud2 
    pcl::PCLPointCloud2 cloud;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*cloud_msg, cloud);

    // cresting  boost shared pointer for pcl function inputs
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZI>
    pcl::fromPCLPointCloud2(cloud, *inputCloudPtr);


    /// First we can apply a pass through filter, so we receive the filtered and the negative filtered cloud
    // create a pcl object to hold the passthrough filtered results
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNegFilteredPtr(new pcl::PointCloud<pcl::PointXYZI>);


    /// Pass Through - Create the filtering object for Surface Detetction
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(inputCloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.5,
                         -1.); // depending on mounting position and relative angle between lidar mount and surface
    pass.filter(*cloudFilteredPtr);
    // apply the negative filter to get the remaining cloud
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloudNegFilteredPtr);  // save remainining cloud

    /// Surface extraction -  perform ransac planar filtration
    /// we receive ground and non ground points

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonGroundCloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg1; // Create the segmentation object
    seg1.setOptimizeCoefficients(true); // Optional
    seg1.setModelType(pcl::SACMODEL_PLANE);  // Mandatory
    seg1.setMethodType(pcl::SAC_MSAC);  // should be faster but less accure than pcl::SAC_RANSAC
    seg1.setDistanceThreshold(0.15);  // take all points in the given threshold
    seg1.setInputCloud(cloudFilteredPtr);
    seg1.segment(*inliers, *coefficients);

    /// * output cloud definition **
    pcl::PCLPointCloud2 outputPCL;
    sensor_msgs::PointCloud2 output;

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    if (inliers->indices.empty()) {
        ROS_INFO_STREAM ("Could not estimate a planar model for the given dataset.");
        *cloudNegFilteredPtr += *cloudFilteredPtr;
    } else {
        // Extract ground plane indices
        extract.setInputCloud(cloudFilteredPtr);
        extract.setIndices(inliers);
        extract.filter(*groundCloud);
        // get remaining cloud
        extract.setNegative(true);
        extract.filter(*nonGroundCloud);
        *cloudNegFilteredPtr += *nonGroundCloud;  // Add remaining cloud

        pcl::toPCLPointCloud2(*groundCloud, outputPCL);
        // Convert to ROS data type
        pcl_conversions::fromPCL(outputPCL, output);
        // add the cluster to the array message
        output.header.frame_id = FRAME_ID;
        pcl_ground.publish(output);


    }
    // In case that we extracted a ground we can apply a pass through for intensities
    if (!groundCloud->points.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr lanePoints(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> passIntensity;
        passIntensity.setInputCloud(groundCloud);
        passIntensity.setFilterFieldName("intensity");
        passIntensity.setFilterLimits(60, 120); // depending on the intensity output of the lidar device
        passIntensity.filter(*lanePoints);

        if (!lanePoints->points.empty()) {
            // convert to sensor message  and output pcd
            pcl::toPCLPointCloud2(*lanePoints, outputPCL);
            pcl_conversions::fromPCL(outputPCL, output);
            // add the cluster to the array message
            output.header.frame_id = FRAME_ID;
            pcl_lane.publish(output);
        } else {
            ROS_INFO_STREAM ("No lane points extracted .");
        }
    }

    if (!cloudNegFilteredPtr->points.empty()) {

        pcl::toPCLPointCloud2(*cloudNegFilteredPtr, outputPCL);
        // Convert to ROS data type
        pcl_conversions::fromPCL(outputPCL, output);
        // add the cluster to the array message
        output.header.frame_id = FRAME_ID;
        pcl_seg.publish(output);

    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";

}

void segmentation::extract_surface(pcl::PointCloud<pcl::PointXYZI>::Ptr) {
    // Place segmentation of surface here
}


void segmentation::extract_objects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    /// * output cloud definition **
    pcl::PCLPointCloud2 outputPCL;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr buildings_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> v = {0, 1, 2, 3}; // Depends on the number of surfaces we're expecting !!
    for (int i : v) {


        pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>::Ptr model_p(new pcl::
        SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>(cloud));
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr vertical_inliers(new pcl::PointIndices);
        pcl::ExtractIndices<pcl::PointXYZI> extract1;

        std::vector<int> in_vertical;

        model_p->setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));
        model_p->setEpsAngle(pcl::deg2rad(2.0));  // need to be set

        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(0.5); // distance threshold of 2m
        ransac.setMaxIterations(500);
        ransac.computeModel(2);
        ransac.getInliers(in_vertical);

        // pcl::copyPointCloud <pcl::PointXYZI> (*combined_cloud, in_vertical, *vertical_pcl);
        vertical_inliers->indices = in_vertical;


        extract1.setInputCloud(cloud);
        extract1.setIndices(vertical_inliers);
        extract1.filter(*tmp_pcl); // write temporary building

        extract1.setNegative(true);
        extract1.filter(*cloud); // substract temp building from infa pcl
        *buildings_pcl += *tmp_pcl;  // add tmp to building cloud


    }
    // publish building cloud
    pcl::toPCLPointCloud2(*buildings_pcl, outputPCL);
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);
    // add the cluster to the array message
    output.header.frame_id = FRAME_ID;
    pcl_objects.publish(output);
}


segmentation::segmentation(ros::NodeHandle nh) : m_nh(nh) {
    // define the subscriber and publisher
    // m_sub = m_nh.subscribe ("cloud_pcd", 1, &segmentation::cloud_callback, this);
    m_sub = m_nh.subscribe("/vlp_102/velodyne_points", 1, &segmentation::cloud_callback, this);
    m_markerArray = m_nh.advertise<visualization_msgs::MarkerArray>("obj_recognition/obj_marker", 1);
    pcl_seg = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_seg/not_segmented", 1);
    pcl_ground = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_seg/ground", 1);
    pcl_objects = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_seg/objects", 1);
    pcl_lane = m_nh.advertise<sensor_msgs::PointCloud2>("cloud_seg/lane", 1);
}
