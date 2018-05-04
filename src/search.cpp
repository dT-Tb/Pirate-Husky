#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <random_numbers/random_numbers.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_sub;          // Subscribe to laser range finder
ros::Publisher pirate_move_pub;     // Publish move commands to the robot

random_numbers::RandomNumberGenerator* rng; // rnjesus for randomwalk simulation

bool searching = 1;  // If the robot is in 'search mode' or not
bool turnLeft = 0;   // Let robot know if it should move to left 
bool turnRight = 0;  // Let robot know if it should move to right 

// Receives the laser scan data
// 
// Will determine whethere there is an obstacle to avoid or not
// If an obstacle is detected, then avoidance is handled immediately
//  and the robot is flagged to stop searching
void LaserHandler(const sensor_msgs::LaserScan& msg)
{
    for(int i = 30; i <= (msg.ranges.size() - 30); i++)
    {
        if(msg.ranges[i] < 1 && msg.ranges[i] > 0.3)
        {
            // We are avoiding obstacles so we stop searching
            searching = 0;
            geometry_msgs::Twist avoid_cmd;
            
            // Turn CW or CCW depending on where the obstacle was detected
            if(i < msg.ranges.size() / 2){
                avoid_cmd.angular.z = 0.0;
                avoid_cmd.linear.x = 0.0;
                pirate_move_pub.publish(avoid_cmd);
                
                avoid_cmd.angular.z = -0.75;
                turnRight = 0;
                turnLeft  = 1;
                
                ROS_INFO_STREAM("Avoiding Left");
            }
            else{
                avoid_cmd.linear.x = 0.0;
                avoid_cmd.angular.z = 0.0;
                pirate_move_pub.publish(avoid_cmd);
                
                avoid_cmd.angular.z = 0.5;
                turnRight = 1;
                turnLeft  = 0;
                
                ROS_INFO_STREAM("Avoiding Right");
            }
            
            pirate_move_pub.publish(avoid_cmd);
            return;
        }
    }
    
    // once obstacle has been avoided then we continue with
    // search protocol
    searching = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "search");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    
    pirate_move_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
    laser_sub = nh.subscribe("/scan", 1000, &LaserHandler);
    rng = new random_numbers::RandomNumberGenerator();
    
    while(ros::ok())
    {
        ros::spinOnce();
        int stop_arc = (int)rng->uniformInteger(1,100);

        // Currently searching and not avoiding
        if(searching) 
        {
            // In 'search mode' we simply move forward slowly and try
            //  to detect things with the logical camera
            geometry_msgs::Twist explore;
        
            // go forward
            explore.linear.x = 0.5;
            explore.angular.z = 0;
            
            // In order to get some more movement around the map, we add
            //  an arc to the robot's movement for a time.
            // 
            // A random number to specify when it will stop turning is used,
            //  and is to make the overall exploration random
            if(turnLeft)
            {
                explore.angular.z = 0.2;
                ROS_INFO_STREAM("Moving Forward: R");
            }
            if(turnRight)
            {
                explore.angular.z = -0.2;
                ROS_INFO_STREAM("Moving Forward: L");
            }
            if(stop_arc == 31)   // If a 27 is randomly generated
            {
                // Stop all arc movements
                explore.angular.z = 0;
                turnLeft = 0;
                turnRight = 0; 
                ROS_INFO_STREAM("Moving Forward: S");
            }

            pirate_move_pub.publish(explore); 
        }
        
        rate.sleep();
    }

    delete rng;
}
