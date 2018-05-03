#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cstdlib>


#define Pi 3.1415926

struct Treasure
{
    double x,y,z;
    bool visited;

    void newTargets()
    {
        if(visited)
        {
            x = (double) (rand()%5 +1)*-1;
            y = (double) (rand()%8 +1)*-1;
            z = 0.1;
            visited = false;
        }
        else
        {
            x = (double) (rand()%8 +1);
            y = (double) (rand()%8 +1);
            z = 0.1;
            visited = true;
        }
    }
};

// std::vector<waypoints> v(4);
Treasure newTarget;
bool intransit = false;
bool avoid;

void generateWaypoints()
{
       newTarget.x = (double)(rand()%8+1);
       newTarget.y = (double)(rand()%7+1);
       newTarget.z = 0.1;
       newTarget.visited = false;
}
void printGoal()
{
    ROS_INFO_STREAM("Target X: " << newTarget.x);
    ROS_INFO_STREAM("Target Y: " << newTarget.y);
    ROS_INFO_STREAM("Target Theta: " << newTarget.z);
}

void laserFeed(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(msg->ranges[90] < 0.5)
    {
        // range = msg->ranges[90];
        avoid = true;
    }
    else
    {
        avoid = false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "huskyExplore");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
    ROS_INFO_STREAM("Waiting on server my dude...");

    ros::Subscriber sub = nh.subscribe("/scan", 1000, laserFeed);
    
    ros::Publisher pub 
      = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",1000);
    
    // generateWaypoints();
    newTarget.newTargets();
    while (!ac.waitForServer()){/*waiting for server to connect*/}  
    
    ros::Rate rate(10);
    if(avoid) {
        ac.cancelAllGoals();
        intransit = false;    
    }
    while(avoid && nh.ok())
    {
         float mm = Pi/6;
         geometry_msgs::Twist movement;
         movement.linear.x = 0.0;
         movement.angular.z = mm;
         mm += Pi/9;
         pub.publish(movement);
         rate.sleep();
         ros::spinOnce();
    }
    while(intransit){
        rate.sleep();
        ros::spinOnce();
    }
    move_base_msgs::MoveBaseGoal waypoint;
    
    waypoint.target_pose.header.frame_id = "map";
    waypoint.target_pose.header.stamp = ros::Time::now();
    waypoint.target_pose.pose.position.x = newTarget.x;
    waypoint.target_pose.pose.position.y = newTarget.y;
    waypoint.target_pose.pose.orientation.w = newTarget.z;
    
    printGoal();
    
    ac.sendGoal(waypoint);
    intransit = true;

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)        
	{
        ROS_INFO_STREAM("Success we made it to the target");
        intransit = false;		
        ac.cancelAllGoals();
    }
	else
    {
        ROS_INFO_STREAM("Failure");
        intransit = false;
        ac.cancelAllGoals();				
    }    
    ros::spin();
    
    return 0;
}

