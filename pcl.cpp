// convex_hull_node.cpp
// Copyright 2014 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
//ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
//std
#include <iostream>
#include <cmath>
//lis
#include <lis_msgs/PosesArray.h>
using namespace std;

class Convex_hull{
    private:
    ros::NodeHandle n;
    ros::Publisher pub, pub2, pub3;
    ros::Subscriber sub;

    public:
    Convex_hull(){
        pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
        pub2 = n.advertise<sensor_msgs::PointCloud2> ("output2", 1);
        pub3 = n.advertise<sensor_msgs::PointCloud2> ("output3", 1);
        sub = n.subscribe("camera/depth/points", 1, &Convex_hull::cloud_cb, this);

    }
    
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
            sensor_msgs::PointCloud2 cloud_box, pub_data, pub_data3;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConcaveHull<pcl::PointXYZ> chull;
            //pcl::PointCloud<pcl::PointXYZ> cloud_data;
            // Perform the actual filtering
            pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
            sor.setInputCloud (cloud);
            sor.setLeafSize (0.1, 0.1, 0.1);
            sor.filter (cloud_box);
            pcl::fromROSMsg (cloud_box, *cloud_data);
            pub2.publish (cloud_box);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud_data);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0, 7.0);
            pass.filter (*cloud_filtered);
            std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            std::cerr << "PointCloud after segmentation has: " << inliers->indices.size () << " inliers." << std::endl;

            // Project the model inliers
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setIndices (inliers);
            proj.setInputCloud (cloud_filtered);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_projected);
            std::cerr << "PointCloud after projection has: " << cloud_projected->points.size () << " data points." << std::endl;
            pcl::toROSMsg (*cloud_projected, pub_data3);
            pub3.publish (pub_data3);
            
            // Create a Concave Hull representation of the projected inliers
            chull.setInputCloud (cloud_projected);
            chull.setAlpha (0.1);
            chull.reconstruct (*cloud_hull);
            pcl::toROSMsg (*cloud_hull, pub_data);
            // Publish the data
            pub.publish (pub_data);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convex_hull_node");

    cout << "Initializing node... " << endl;
    Convex_hull convex_hull;
    ros::spin();
    
    return 0;
}
