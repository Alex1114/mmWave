#include <iostream>
// Ros
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>

using namespace ros;
using namespace pcl;
using namespace std;

class icp_loc{
  public:
	icp_loc();
	void pc0_callback(const sensor_msgs::PointCloud2 msg);    // sensor_msgs::PointCloud2
	void pc1_callback(const sensor_msgs::PointCloud2 msg);    // sensor_msgs::PointCloud2
	void pose_callback(const geometry_msgs::Pose msg);    // sensor_msgs::PointCloud2

  private:
	Subscriber pose_sub;
	Subscriber pc_sub0;
	Subscriber pc_sub1;
	Publisher pc_map;
	Publisher pose_pub_client;
	geometry_msgs::PoseArray posearr_msg;

	PointCloud<PointXYZ>::Ptr map;
	PointCloud<PointXYZ>::Ptr map_process;
	PointCloud<PointXYZ>::Ptr pc_input0;
	PointCloud<PointXYZ>::Ptr pc_input1;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr result;
	int count;
	VoxelGrid<PointXYZ> downsample;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	sensor_msgs::PointCloud2 ros_cloud_msg;
	sensor_msgs::PointCloud2 origin_map;

	Publisher marker_pub;
	uint32_t shape;
	visualization_msgs::Marker marker;
};