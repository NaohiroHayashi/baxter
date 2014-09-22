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
#include <pcl/filters/passthrough.h>
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
    ros::Publisher pub3, pub4;
    ros::Subscriber sub;
    sensor_msgs::PointCloud2 pub_filetrerd_xyz, pub_convex_full;
    
    public:
    Convex_hull(){
        pub3 = n.advertise<sensor_msgs::PointCloud2> ("convex_hull", 1);
        pub4 = n.advertise<sensor_msgs::PointCloud2> ("xyz_filter", 1);
        sub = n.subscribe("camera/depth/points", 1, &Convex_hull::cloud_convex, this);
    }
    
    void cloud_convex (const sensor_msgs::PointCloud2ConstPtr& msg)
    {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> chull;

            //filtering_xyz
            pcl::fromROSMsg (*msg, *ros_cloud);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (ros_cloud); pass.setFilterFieldName ("z"); pass.setFilterLimits (0, 0.8); pass.filter (*cloud_filtered_z);//z
            pass.setInputCloud (cloud_filtered_z); pass.setFilterFieldName ("y"); pass.setFilterLimits (0.0, 0.0015); pass.filter (*cloud_filtered_y);//y
            pass.setInputCloud (cloud_filtered_y); pass.setFilterFieldName ("x"); pass.setFilterLimits (-0.2, 0.2); pass.filter (*cloud_filtered_x);//x
            std::cerr << "PointCloud after filtering has: " << cloud_filtered_x->points.size () << " data points." << std::endl;
            pcl::toROSMsg (*cloud_filtered_x, pub_filetrerd_xyz);
            pub4.publish (pub_filetrerd_xyz);

            // Create a ConvexHull representation of the projected inliers
            chull.setInputCloud (cloud_filtered_x);
            chull.reconstruct (*cloud_hull);
            pcl::toROSMsg (*cloud_hull, pub_convex_full);
            pub3.publish (pub_convex_full);
            
            //calculation Convex rectangle
            cal_rectangle(*cloud_hull);
    }
    
    void cal_rectangle(pcl::PointCloud<pcl::PointXYZ>& cloud){
        std::cout << cloud.width << std::endl;
        int max_z, min_z;
        
        for(int i=0; i<cloud.width; i++){
            if(i==0){
                max_z = cloud.points[i].z;
                min_z = cloud.points[i].z;
            }
            else if(cloud.points[i].z > max_z){
                max_z = cloud.points[i].z;
            }
            else if(cloud.points[i].z < min_z){
                min_z = cloud.points[i].z;
            }
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
