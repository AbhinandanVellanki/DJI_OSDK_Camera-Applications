#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include <Eigen/Core>

//using  namespace  Eigen;


//Eigen::Matrix<float, 4, 4> transform (new Eigen::Matrix<float, 4, 4> ());
Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

ros::Publisher transformed_cloud_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& raw_cloud){
	transform(0,0) = -0.79350503;
	transform(0,1) = -0.19572782;
	transform(0,2) = -0.57622946;
	transform(1,0) = 0.12830843;
	transform(1,1) = -0.97939111;
	transform(1,2) = 0.15598074;
	transform(2,0) = -0.59488378;
	transform(2,1) = 0.4983640;
	transform(2,2) = 0.80226530;
	transform(0,3) = 0.68003865;
	transform(1,3) = -14.18108947;
	transform(2,3) = 6.2781987;

	pcl::PointCloud<pcl::PointXYZ> source_cloud;
	//pcl_conversions::toPCL(raw_cloud, source_cloud);
	pcl::fromROSMsg(*raw_cloud, source_cloud);

	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

	pcl::transformPointCloud (source_cloud, transformed_cloud, transform);

	sensor_msgs::PointCloud2::Ptr new_cloud (new sensor_msgs::PointCloud2);
	pcl::toROSMsg(source_cloud, *new_cloud);

	new_cloud->header = raw_cloud->header;
	transformed_cloud_pub.publish(new_cloud);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "point_cloud_transformer");
	ros::NodeHandle n;

	//publisher for projected point cloud
	transformed_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("my_point_cloud/main_cam", 10);

	//subscriber for unprojected point cloud
	ros::Subscriber raw_cloud_sub = n.subscribe("/stereo_depth_perception/unprojected_pt_cloud", 10, callback);

	ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}