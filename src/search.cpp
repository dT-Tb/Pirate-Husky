#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_sub;
ros::Publisher pirate_move_pub;

bool searching = 1;

// Receives the laser scan data
// 
// Will determine whethere there is an obstacle to avoid or not
// If an obstacle is detected, then avoidance is handled immediately
//  and the robot is flagged to stop searching
void LaserHandler(const sensor_msgs::LaserScan& msg)
{
    for(int i = 30; i <= (msg.ranges.size() - 30); i++)
    {
        if(msg.ranges[i] < 1)
        {
            searching = 0;
            geometry_msgs::Twist avoid_cmd;
            
            if(i < msg.ranges.size() / 2){
                avoid_cmd.angular.z = -0.5;
                ROS_INFO_STREAM("Avoiding Left");
            }
            else{
                avoid_cmd.angular.z = 0.5;
                ROS_INFO_STREAM("Avoiding Right");
            }

            pirate_move_pub.publish(avoid_cmd);
            return;
        }
    }

    searching = 1;
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
        
        // Currently searching and not avoiding
        if(searching) 
        {
            // do the thing
            geometry_msgs::Twist explore;
            explore.linear.x = 0.75;
            explore.angular.z = 0;

            pirate_move_pub.publish(explore); 
        } 
        
        rate.sleep();
    }
}
