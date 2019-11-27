#include "orb_process_base.h"

icp_loc::icp_loc(){
	NodeHandle nh;
	new_map.reset(new PointCloud<PointXYZ>());
	pc_filter_input0.reset(new PointCloud<PointXYZ>());
	scene_cloud_.reset(new PointCloud<PointXYZ>());
	std::vector<PointCloud<PointXYZ>> source_clouds;

    posearr_msg = geometry_msgs::PoseArray();
	posearr_msg.header = std_msgs::Header();

    pc_map = nh.advertise<sensor_msgs::PointCloud2> ("/mmwave_slam_mapping", 10);
	pc_sub0 = nh.subscribe("/filtered_pc", 1, &icp_loc::pc0_callback, this);
	
}

void icp_loc::pc0_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_filter_input0);

	tf::StampedTransform transform;
	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("/slam_map", "/base_link", ros::Time(0), five_seconds);
		listener.lookupTransform("/slam_map", "/base_link", ros::Time(0), transform);
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
	pcl::transformPointCloud (*pc_filter_input0, *pc_filter_input0, trans);

	*new_map += *pc_filter_input0;
	//slam

	toROSMsg(*new_map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "/base_link";
	pc_map.publish(ros_cloud_msg);
}


int main(int argc, char** argv){
	init(argc, argv, "orb_process_slam");
	icp_loc icp_loc;
	spin();
	return 0;
}