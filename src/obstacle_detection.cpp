#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

ros::Subscriber laser_sub;
ros::Publisher laser_pub;

void LaserHandler(const sensor_msgs::LaserScan& msg)
{
    float frontrange = msg.ranges[msg.ranges.size() / 2];
    std_msgs::Float32 straightAhead;
    straightAhead.data = frontrange;
    laser_pub.publish(straightAhead);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    laser_pub = nh.advertise<std_msgs::Float32>("/collisionDetection", 1000);
    laser_sub = nh.subscribe("/scan", 1000, &LaserHandler);

    ros::spin();
}
