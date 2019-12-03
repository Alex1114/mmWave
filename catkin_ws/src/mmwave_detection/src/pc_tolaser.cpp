#include "pc_tolaser.h"

// Constructor
Pctolaser::Pctolaser(ros::NodeHandle *nh) : nh_(*nh)
{
    last_time = ros::Time::now();
    pub_laser = nh_.advertise<sensor_msgs::LaserScan>("RL/scan/mmwave", 1);
    sub_mmwave_pc = nh_.subscribe("/filtered_pc", 1, &Pctolaser::mmwave_data_cb, this);
    ROS_INFO("start node %s",ros::this_node::getName().c_str());
}

bool compareByLength(const laser &a, const laser &b)
{
    return a.angle < b.angle;
}

void Pctolaser::mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_msg)
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    // copyPointCloud(*scene_cloud_, *cloud);
    pcl::fromROSMsg(*in_cloud_msg, *cloud);

    // to laser scan
    vector<laser> lc_data;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        laser ls;
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        ls.dis = sqrt(pow(x, 2) + pow(y, 2));
        ls.angle = atan2(y, x);
        lc_data.push_back(ls);
    }

    // sample from laser
    float max_dis = 5;
    float min_anlge = -3.14159;
    float max_angle = 3.14159;
    float angle_increment = 0.02;
    float start_a = min_anlge;
    sort(lc_data.begin(), lc_data.end(), [](laser a, laser b) { return a.angle < b.angle; });

    vector<float> dis;
    while (!lc_data.empty())
    {
        float td = 0, c = 0;
        while (true)
        {
            if (lc_data[0].angle > start_a)
            {
                if (lc_data[0].angle > start_a + angle_increment)
                {
                    if (c == 0)
                    {
                        td = max_dis;
                        c = 1;
                    }
                    break;
                }
                else
                {
                    c++;
                    td += lc_data[0].dis;
                    lc_data.erase(lc_data.begin());
                }
            }
            else
            {
                break;
            }
        }
        if (c != 0)
            dis.push_back(td / c);
        start_a += angle_increment;
    }

    //to LaserScan.msg
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "base_link";
    scan.angle_increment = angle_increment;
    scan.angle_min = min_anlge;
    scan.angle_max = max_angle;
    scan.scan_time = (last_time - scan.header.stamp).toSec();
    last_time = ros::Time::now();
    scan.range_max = max_dis;
    scan.range_min = 0;
    scan.ranges = dis;
    pub_laser.publish(scan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmwaveToLaser");
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Pctolaser node(&nh);
    ros::spin();
    return 0;
}