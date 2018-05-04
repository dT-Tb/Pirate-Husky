#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_sub;          // Subscribe to laser range finder
ros::Publisher pirate_move_pub;     // Publish move commands to the robot

bool searching = 1;     // If the robot is in 'search mode' or not

// Receives the laser scan data
// 
// Will determine whethere there is an obstacle to avoid or not
// If an obstacle is detected, then avoidance is handled immediately
//  and the robot is flagged to stop searching
void LaserHandler(const sensor_msgs::LaserScan& msg)
{
    for(int i = 45; i <= (msg.ranges.size() - 45); i++)
    {
        if(msg.ranges[i] < 1)
        {
            // We are in 'avoidance mode' and not 'search mode'
            searching = 0;
            geometry_msgs::Twist avoid_cmd;
            
            // Turn CW or CCW depending on where the obstacle was detected
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
            // In 'search mode' we simply move forward slowly and try
            //  to detect things with the logical camera
            geometry_msgs::Twist explore;
            explore.linear.x = 0.5;
            explore.angular.z = 0;

            pirate_move_pub.publish(explore); 
        } 
        
        rate.sleep();
    }
}
