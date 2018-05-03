#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>


ros::Publisher pub;
random_numbers::RandomNumberGenerator* RNGJESUS;

double px,py,pz;
double x, y, z;

void GenPoints()
{
    if(ros::ok())
    {
        geometry_msgs::Pose2D waypoint;
        RNGJESUS = new random_numbers::RandomNumberGenerator();
        
        x = (double) RNGJESUS->uniformInteger(-8,8);
        y = (double) RNGJESUS->uniformInteger(-8,8);
        z = 0.1;
        
        waypoint.x = x; px = x;
        waypoint.y = y; py = y;
        waypoint.theta = z; pz = z;

        pub.publish(waypoint);
    }
}
void maybe(const std_msgs::Bool & msg)
{
    if(msg.data)
    {
        GenPoints();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "points");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Pose2D>("newPoints", 1000);
    ros::Subscriber sub = nh.subscribe("/MORE", 1000, maybe);
    while (nh.ok())
    {
        ros::spinOnce();
        GenPoints();
    }
    
    ros::spin();
    return 0;
}
