#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_sub;
ros::Publisher pirate_move_pub;

bool searching = 1;
bool leftSideClose = 0;
bool rightSideClose = 0;

void LaserHandler(const sensor_msgs::LaserScan& msg)
{
    for(int i = 60; i < msg.ranges.size()/2; i++)
    {
        if(msg.ranges[i] < 1.5)
        {
            searching = 0;
            leftSideClose = 1;
            ROS_INFO_STREAM("Avoiding Left");
            goto yeboi;
        }
    }
	
	for(int i = msg.ranges.size()/2; i <= 120; i++)
    {
        if(msg.ranges[i] < 1.5)
        {
            searching = 0;
            rightSideClose = 1;
            ROS_INFO_STREAM("Avoiding Left");
            goto yeboi;
        }
    }

yeboi:
    if(leftSideClose || rightSideClose){
        searching = 0;
        ROS_INFO_STREAM("Avoidance active");
    }
    else{
        searching = 1;
        ROS_INFO_STREAM("Searching active");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "search");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    
    pirate_move_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
    laser_sub = nh.subscribe("/scan", 1000, &LaserHandler);
    
    while(ros::ok())
    {
        ros::spinOnce();

        if(searching)   // Looking for treasures
        {
            // do the thing
            geometry_msgs::Twist explore;
            explore.linear.x = 0.75;
            explore.angular.z = 0;

            pirate_move_pub.publish(explore); 
        }
        else    // Obstacle avoidance
        {
            // dont hit the thing 
            geometry_msgs::Twist avoid;
            avoid.linear.x = 0;
            if(leftSideClose && rightSideClose)
            {
                avoid.angular.z = 0.5;
                leftSideClose = 0;
                rightSideClose = 0;
                ROS_INFO_STREAM("both");
            }
            else if(leftSideClose){
            	avoid.angular.z = -0.5;
            	leftSideClose = 0;
                ROS_INFO_STREAM("left");
            }
            else if(rightSideClose){
            	avoid.angular.z = 0.5;
            	rightSideClose = 0;
                ROS_INFO_STREAM("right");
            }
            else{
            	avoid.angular.z = 0.5;
                ROS_INFO_STREAM("none");
            }
             
            pirate_move_pub.publish(avoid);
        }
        
        rate.sleep();
    }
}
