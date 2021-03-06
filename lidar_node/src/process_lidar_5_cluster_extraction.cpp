#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/kdtree/kdtree.h>
#include "pcl_ros/point_cloud.h"
#include <tf/transform_broadcaster.h>

class SubscribeProcessPublish
{
public:
    void SubscribeProcessPublish1()
    {
        // Assign subscriber
        this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurement, this);
        
	// Assign publisher
        this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("/velodyne_points_filtered", 5); 
        this->publisher_plane = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/velodyne_segmented_plane", 5); 
        //this->publisher_plane = this->nh.advertise<pcl::PCLPointCloud2> ("/velodyne_segmented_plane", 1);      
    }
    
        void processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg)
        {

        while (nh.ok()){

        std::cout << "Received lidar measurement with seq ID " << cloud_msg->header.seq << std::endl;

        // Publish the data
        //this->publisher.publish (cloud_msg);
        std::cerr << "PointCloud before filtering: " << cloud_msg->width * cloud_msg->height 
             << " data points (" << pcl::getFieldsList (*cloud_msg) << ")." << std::endl;



        ////// Down-sampling the pointcloud ///////
        //pcl::PCLPointCloud2::Ptr cloud_msg (new pcl::PCLPointCloud2 ());
        // define a new container for the data
        pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
        // define a voxelgrid
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;

        // set input to cloud
        voxelGrid.setInputCloud(cloud_msg);
        // set the leaf size (x, y, z)
        voxelGrid.setLeafSize(0.05, 0.05, 0.05); // 5cm leaf size
        // apply the filter to dereferenced cloudVoxel
        voxelGrid.filter(*cloudVoxel);

        // Publish the data for visualisation
        //this->publisher.publish (*cloudVoxel);
        std::cerr << "PointCloud after filtering: " << cloudVoxel->width * cloudVoxel->height 
             << " data points (" << pcl::getFieldsList (*cloudVoxel) << ")." << std::endl;



        /////// Filtering: removing the floor using PassThrough and specifying the x, y, z limits //////
        // cascade the floor removal filter and define a container for floorRemoved	
        pcl::PCLPointCloud2::Ptr xFiltered (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr yFiltered (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());		
        // define a PassThrough filter
        pcl::PassThrough<pcl::PCLPointCloud2> pass;

        // limits in x direction
        pass.setInputCloud(cloudVoxel);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-2.0, 17.0);
        pass.filter(*xFiltered);

        // limits in y direction
        pass.setInputCloud(xFiltered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-5.0, 5.0);
        pass.filter(*yFiltered);

        // limits in z direction
        pass.setInputCloud(yFiltered);
        pass.setFilterFieldName("z");
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



        ////// START OF PLANE SEGMENTATION
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(pclXYZ));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (200);
        seg.setDistanceThreshold (0.004);
        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud (cropped_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
           std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          //break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        for (std::size_t i = 0; i < cloud_plane->points.size (); ++i)
          std::cerr << "    " << cloud_plane->points[i].x << " "
                              << cloud_plane->points[i].y << " "
                              << cloud_plane->points[i].z << std::endl;

        this->publisher_plane.publish(*cloud_plane);

        //pcl::PCLPointCloud2 cloud_plane2;
        //toPCLPointCloud2(*cloud_plane, cloud_plane2);
        //this->publisher_plane.publish(*cloud_plane2);
        ////// END OF PLANE SEGMENTATION //////
     
        // if 1 or more points were utilised, consider the centroid to be valid
        if(pcl::computeCentroid(pclXYZ, centroid) > 0)
        {
            try
            {
            // define orientation which will be sent to robot
            double orientation = atan2 (centroid.y, centroid.x);
            std::cout << "Centroid x: " << centroid.x << " y: " << centroid.y << " z: " << centroid.z << " with orientation " << orientation*57.2957795 << std::endl;

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(centroid.x, centroid.y, centroid.z) );
            tf::Quaternion q;
            q.setRPY(0, 0, orientation);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "supply"));
            }
            catch (tf::TransformException ex)
            {
              ROS_ERROR_STREAM(ex.what());
            }
        }



        ////// Cluster extraction ///////
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_plane);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1); // 10cm
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_plane);
        ec.extract (cluster_indices);
        std::cout << "Check1" << std::endl;

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          std::cout << "Check2" << std::endl;
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
              cloud_cluster->points.push_back (cloud_plane->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          pcl::PCDWriter writer; 
          std::stringstream ss;
          ss << "cloud_cluster_" << j << ".pcd";
          writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
          j++;
        }

        } //while

    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::Publisher publisher_plane;
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
