#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_sub;          // Subscribe to laser range finder
ros::Subscriber goal_status_sub;    // Subscribe to the current goal state
ros::Publisher pirate_move_pub;     // Publish move commands to the robot
ros::Publisher obst_det_pub;        // Publish state of obstacle detection
ros::Timer search_cutoff;           // Timer to cutoff 'search mode'

bool searching = 0;     // If the robot is in 'search mode' or not
bool travelling = 1;    // If planner is working towards a goal state

// Tells the robot to stop 'search mode' and initiate planner again
void cutoffTimerHandler(const ros::TimerEvent&)
{
   searching = 0;
   travelling = 1;

   // probably need to publish to a topic here so that the robot
   //   knows to initiate the planner again.
}

// Check whether the robot has reached its goal or not
void GoalStateReceived(const std_msgs::Bool& msg)
{
    // Should only change travelling to 'false' most of the time
    //  
    // When goal is reached it will return 'false' as well
    travelling = msg.data;
}

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
            // If obstacle detected, then we publish to a topic to 
            //  describe the current state to the planner node
            std_msgs::Bool obstacle_detected;
            obstacle_detected.data = 1;
            obst_det_pub.publish(obstacle_detected);
            
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
    
    // If we were previously in 'search mode' then we restore that mode
    //  and tell the planner that the robot's good to go
    if(!travelling){
        // std_msgs::Bool obstacle_detected;
        // obstacle_detected.data = 0;
        // obst_det_pub.publish(obstacle_detected);
        
        searching = 1;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "search");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    
    pirate_move_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
    obst_det_pub = nh.advertise<std_msgs::Bool>("/obstacleDetected", 1000);
    
    goal_status_sub = nh.subscribe("/goalStatus", 1000, &GoalStateReceived);
    laser_sub = nh.subscribe("/scan", 1000, &LaserHandler);
    
    search_cutoff = nh.createTimer( ros::Duration(30.0), cutoffTimerHandler );

    while(ros::ok())
    {
        ros::spinOnce();
        
        // Currently searching and not avoiding
        if(searching && !travelling) 
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
