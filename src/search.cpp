#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber obst_det_sub;
bool searching = 1;

void obstacleAvoidance(const std_msgs::Float32& msg)
{
    if(msg.data < 0.5)
        searching = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "search");
    ros::NodeHandle nh;

    obst_det_sub = nh.subscribe("/collisionDetection", 1000, &obstacleAvoidance);
    ros::spin();
}
