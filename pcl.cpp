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
//std
#include <iostream>
#include <cmath>
//lis
#include <lis_msgs/PosesArray.h>
using namespace std;

class Convex_hull{
    private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    
    public:
    Convex_hull(){
        pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
        sub = n.subscribe("camera/depth/points", 1, &Convex_hull::cloud_cb, this);
    }
    
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
          sensor_msgs::PointCloud2 cloud_filtered;

          // Perform the actual filtering
          pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
          sor.setInputCloud (cloud);
          sor.setLeafSize (0.03, 0.03, 0.03);
          sor.filter (cloud_filtered);

          // Publish the data
          pub.publish (cloud_filtered);
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
