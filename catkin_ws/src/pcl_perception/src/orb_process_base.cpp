#include "orb_process_base.h"

icp_loc::icp_loc(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input0.reset(new PointCloud<PointXYZ>());
	pc_input1.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());

	posearr_msg = geometry_msgs::PoseArray();
	posearr_msg.header = std_msgs::Header();

	shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	//marker_pub = nh.advertise<visualization_msgs::Marker>("/slam_path_base", 1);
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;
	count = 0;
	downsample.setLeafSize (0.6f, 0.6f, 0.6f);
	// Ros 0
	// pc_process = nh.advertise<sensor_msgs::PointCloud2> ("/pc_part_map", 10);
	//pose_pub_client = nh.advertise<geometry_msgs::PoseArray> ("/poses", 10);
	pc_map = nh.advertise<sensor_msgs::PointCloud2> ("/mmwave_mapping", 10);
	pc_sub0 = nh.subscribe("filtered_pc", 1, &icp_loc::pc0_callback, this);
	pc_sub1 = nh.subscribe("/ti_mmwave/radar_scan_pcl_1", 1, &icp_loc::pc1_callback, this);
	//pose_sub = nh.subscribe("/move_base_simple/goal", 1, &icp_loc::pose_callback, this);

}

void icp_loc::pc0_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input0);
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

	pcl::transformPointCloud (*pc_input0, *pc_input0, trans);

	*map += *pc_input0;

	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = msg.header.frame_id;
	pc_map.publish(ros_cloud_msg);
	
	/* 
	if (count % 100 == 0){
		fromROSMsg (msg, *pc_input);
		*map += *pc_input;

		toROSMsg(*map, ros_cloud_msg);
		ros_cloud_msg.header.frame_id = "slam_map";
		pc_map.publish(ros_cloud_msg);
	}
	*/
	// if(count % 50 == 0){
	// 	*map += *pc_filter;
	// 	cout << trans << endl;
	// }

	// if(count % 500 == 0){
	// downsample.setInputCloud (map);
	// downsample.filter (*map);
	// }
	// geometry_msgs::Point p;
	// p.x = transform.getOrigin().x();
	// p.y = transform.getOrigin().y();
	// p.z = transform.getOrigin().z();
	// marker.points.push_back(p);
	// marker.lifetime = ros::Duration();
	// marker_pub.publish(marker);

	count ++;

}

void icp_loc::pc1_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input1);
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

	pcl::transformPointCloud (*pc_input1, *pc_input1, trans);

	*map += *pc_input1;

	//toROSMsg(*map, ros_cloud_msg);
	//ros_cloud_msg.header.frame_id = msg.header.frame_id;
	//pc_map.publish(ros_cloud_msg);
	count ++;

}

/*void icp_loc::pose_callback(const geometry_msgs::Pose msg){

	posearr_msg.poses.push_back(msg);
	if(posearr_msg.poses.size()>=50){
		pose_pub_client.publish(posearr_msg);
		posearr_msg = geometry_msgs::PoseArray();
		posearr_msg.header = std_msgs::Header();
	}
}
*/

int main(int argc, char** argv){
	init(argc, argv, "orb_process_base");
	icp_loc icp_loc;
	spin();
	return 0;
}
