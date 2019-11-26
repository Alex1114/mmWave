#include "orb_process_base.h"

icp_loc::icp_loc(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	//map_process.reset(new PointCloud<PointXYZ>());
	pc_input0.reset(new PointCloud<PointXYZ>());
	pc_input1.reset(new PointCloud<PointXYZ>());
	pc_input2.reset(new PointCloud<PointXYZ>());
	pc_input3.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	//result.reset(new PointCloud<PointXYZ>());
	scene_cloud_.reset(new PointCloud<PointXYZ>());
	std::vector<PointCloud<PointXYZ>> source_clouds;

    posearr_msg = geometry_msgs::PoseArray();
	posearr_msg.header = std_msgs::Header();

	//shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;

    pc_map = nh.advertise<sensor_msgs::PointCloud2> ("/mmwave_mapping", 10);
	pc_sub0 = nh.subscribe("/ti_mmwave/radar_scan_pcl_0", 1, &icp_loc::pc0_callback, this);
	pc_sub1 = nh.subscribe("/ti_mmwave/radar_scan_pcl_1", 1, &icp_loc::pc1_callback, this);
    pc_sub2 = nh.subscribe("/ti_mmwave/radar_scan_pcl_2", 1, &icp_loc::pc2_callback, this);
    pc_sub3 = nh.subscribe("/ti_mmwave/radar_scan_pcl_3", 1, &icp_loc::pc3_callback, this);
	
}
void icp_loc::timerCallback(const ros::TimerEvent& event)
{
	*map += *pc_input0;
	*map += *pc_input1;
	*map += *pc_input2;
	*map += *pc_input3;
	
	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "/base_link";
	pc_map.publish(ros_cloud_msg);
	ROS_INFO("ok");
	map->points.clear();

}

void icp_loc::pc0_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input0);

	tf::StampedTransform transform;
	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("/base_link", "/mmwave_right_link", ros::Time(0), five_seconds);
		listener.lookupTransform("/base_link", "/mmwave_right_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	Eigen::Matrix4f trans;
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	pcl::transformPointCloud (*pc_input0, *pc_input0, trans);
}

void icp_loc::pc1_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input1);

	tf::StampedTransform transform;
	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("/base_link", "/mmwave_left_link", ros::Time(0), five_seconds);
		listener.lookupTransform("/base_link", "/mmwave_left_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	Eigen::Matrix4f trans;
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	pcl::transformPointCloud (*pc_input1, *pc_input1, trans);
}

void icp_loc::pc2_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input2);

	tf::StampedTransform transform;
	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("/base_link", "/mmwave_right_link_back", ros::Time(0), five_seconds);
		listener.lookupTransform("/base_link", "/mmwave_right_link_back", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	Eigen::Matrix4f trans;
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	pcl::transformPointCloud (*pc_input2, *pc_input2, trans);
}
void icp_loc::pc3_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input3);

	tf::StampedTransform transform;
	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("/base_link", "/mmwave_left_link_back", ros::Time(0), five_seconds);
		listener.lookupTransform("/base_link", "/mmwave_left_link_back", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	Eigen::Matrix4f trans;
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	pcl::transformPointCloud (*pc_input3, *pc_input3, trans);
}


int main(int argc, char** argv){
	init(argc, argv, "orb_process_base");
	icp_loc icp;
	NodeHandle nh;
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), &icp_loc::timerCallback, &icp);
	spin();
	return 0;
}