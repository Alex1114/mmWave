#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
// ROS related
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
// PCL related
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace pcl;

typedef struct
{
    float dis;
    float angle;
}laser;


class Pctolaser{
public:
    Pctolaser(ros::NodeHandle* nh);
private:
    void mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_laser;
    ros::Subscriber sub_mmwave_pc;

    ros::Time last_time;
    

};
