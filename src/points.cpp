#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>


ros::Publisher pub;
random_numbers::RandomNumberGenerator* RNGJESUS;

double px,py,pz;
double x, y, z;
bool stop = false;

void GenPoints()
{
    if(ros::ok())
    {
        geometry_msgs::Pose2D waypoint;
        RNGJESUS = new random_numbers::RandomNumberGenerator();
        
        x = (double) RNGJESUS->uniformInteger(-5,5);
        y = (double) RNGJESUS->uniformInteger(-5,5);
        z = 0.1;
        ROS_INFO_STREAM("Pub x:"<< x);
        ROS_INFO_STREAM("Pub y:"<< y);
        ROS_INFO_STREAM("Pub t:"<< z);
        waypoint.x = x; px = x;
        waypoint.y = y; py = y;
        waypoint.theta = z; pz = z;

        pub.publish(waypoint);
    }
}
void subCallback(const std_msgs::Bool & tmp)
{
    if(tmp.data == true)
    {
        stop = true;
    }
}
void maybe(const std_msgs::Bool & msg)
{
    if(msg.data == true)
    {
        GenPoints();
    }
    else 
    {
        return;
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "points");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Pose2D>("Waypoints", 1000);
    ros::Subscriber sub = nh.subscribe("/MORE", 1000, maybe);
    ros::Subscriber noMore = nh.subscribe("NoMore", 1000, subCallback);
    
    ros::Rate rate(10);
    while(nh.ok()&& !stop)
    {
        GenPoints();
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();

    return 0;
}
