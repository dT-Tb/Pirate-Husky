#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <random_numbers/random_numbers.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
ros::Subscriber laser_sub;          // Subscribe to laser range finder
ros::Subscriber goal_status_sub;    // Subscribe to the current goal state
ros::Publisher pirate_move_pub;     // Publish move commands to the robot
ros::Publisher obst_det_pub;        // Publish state of obstacle detection
ros::Timer search_cutoff;           // Timer to cutoff 'search mode'

random_numbers::RandomNumberGenerator *RNG;

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
                // avoid_cmd.linear.x = 0.0;
                ROS_INFO_STREAM("Avoiding Left");
            }
            else{
                // avoid_cmd.linear.x = 0.0;
                avoid_cmd.angular.z = 0.5;
                ROS_INFO_STREAM("Avoiding Right");
            }

            pirate_move_pub.publish(avoid_cmd);
            return;
        }
    }
    // once obstacel has been avoided then we continue with
    // search protocol
    searching = 1;
}
void moveLeft()
{
    geometry_msgs::Twist Left;
    ROS_INFO_STREAM("RNG:Moving Left");
    for(int i =0; i < 100; i ++)
    {
        // Left.linear.x = 0.0;
        Left.angular.z = -0.5;
        pirate_move_pub.publish(Left);
    }
}

void moveRight()
{
    geometry_msgs::Twist Right;
    ROS_INFO_STREAM("RNG:Moving Right");
    for(int i =0; i < 100; i ++)
    {
        // Right.linear.x = 0.0;
        Right.angular.z = 0.5;
        pirate_move_pub.publish(Right);
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
        
        // Currently searching and not avoiding
        if(searching) 
        {
            RNG = new random_numbers::RandomNumberGenerator();
            // In 'search mode' we simply move forward slowly and try
            //  to detect things with the logical camera
            geometry_msgs::Twist explore;
            int tmp1, tmp2;
            tmp1 = rand();
            if(tmp1 % 23 == 0)
            {
                // move the robot to the right
                moveRight();
            }
            else if(tmp1 % 15 == 0)
            {
                // move the robot left and go straight
                moveLeft();
            }
            else 
            {
                // go forward
                ROS_INFO_STREAM("Moving Forward");
                explore.linear.x = 0.5;
                explore.angular.z = 0;
                pirate_move_pub.publish(explore); 

            }
        } 
        
        rate.sleep();
    }
}
