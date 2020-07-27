#include "ros/ros.h"
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>

class SubscribeProcessPublish
{
public:
    void SubscribeProcessPublish1()
    {
        // Assign subscriber
        this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurement, this);
        
	// Assign publisher
        this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("/lidar_output", 1);
    }
    
        void processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg)
        {

        while (nh.ok()){

        std::cout << "Received lidar measurement with seq ID " << cloud_msg->header.seq << std::endl;

        // Publish the data
        this->publisher.publish (cloud_msg);

        ////// Down-sampling the pointcloud ///////
        //pcl::PCLPointCloud2::Ptr cloud_msg (new pcl::PCLPointCloud2 ());
        // define a new container for the data
        pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
        // define a voxelgrid
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;

        // set input to cloud
        voxelGrid.setInputCloud(cloud_msg);
        // set the leaf size (x, y, z)
        voxelGrid.setLeafSize(0.1, 0.1, 0.1);
        // apply the filter to dereferenced cloudVoxel
        voxelGrid.filter(*cloudVoxel);

        // Publish the data for visualisation
        this->publisher.publish (*cloudVoxel);

        /////// Filtering: removing the floor using PassThrough and specifying the vertical range //////
        // cascade the floor removal filter and define a container for floorRemoved	
        pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());		
        // define a PassThrough filter
        pcl::PassThrough<pcl::PCLPointCloud2> pass;

        // set input to cloudVoxel
        pass.setInputCloud(cloudVoxel);
        // filter along z-axis
        pass.setFilterFieldName("z");
        // set z-limits
        pass.setFilterLimits(-0.833, 0.0);
        pass.filter(*floorRemoved);

        // Publish the data for visualisation
        this->publisher.publish (*floorRemoved);

        ////// Finding the centroid and orientation, wrt to lidar frame ///////
        // define a point cloud of type PointXYZ
        pcl::PointCloud<pcl::PointXYZ> pclXYZ;
        // copy the contents of the floorRemoved to pclXYZ
        pcl::fromPCLPointCloud2(*floorRemoved, pclXYZ);
        pcl::PointXYZ centroid;
        
        // if 1 or more points were utilised, consider the centroid to be valid
        if(pcl::computeCentroid(pclXYZ, centroid) > 0)
        {
            // define orientation which will be sent to robot
            double orientation = atan2 (centroid.y, centroid.x);
            std::cout << "Centroid x: " << centroid.x << " y: " << centroid.y << " z: " << centroid.z << " with orientation " << orientation*57.2957795 << std::endl;
        }

        } //while

    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
};

int main(int argc, char** argv)
{
    // initialise the node
    ros::init(argc, argv, "process_lidar");

    std::cout << "Process_lidar node initialised" << std::endl;

    SubscribeProcessPublish SubscribeProcessPublish_;
    SubscribeProcessPublish_.SubscribeProcessPublish1();

    // handle ROS communication events
    //while (ros::ok()) {
    //		//ros::spinOnce();
    //            ros::spin();
    //}

    ros::spin();

    return 0;
}
