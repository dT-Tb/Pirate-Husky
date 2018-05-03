#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/Bool.h"
#include <vector>
#include <cstdlib>


#define Pi 3.1415926

bool intransit = false;
// bool avoid;
bool wait;

double trgt_X, trgt_Y, trgt_T;

void gotToPose(const geometry_msgs::Pose2D msg)
{
    trgt_X = msg.x;
    trgt_Y = msg.y;
    trgt_T = msg.theta;
    wait = true;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "huskyExplore");
    ros::NodeHandle nh;

    ros::Subscriber pointSub = nh.subscribe("/MORE", 1000, gotToPose);
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("anotherOne", 1000);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
    ROS_INFO_STREAM("Waiting on server my dude...");

    // ros::Subscriber sub = nh.subscribe("/scan", 1000, laserFeed);

    while (!ac.waitForServer()){/*waiting for server to connect*/}  
    std_msgs::Bool ARG;
    
    ros::Rate rate(10);
    
    while(!wait){
        rate.sleep();
        ros::spinOnce();
    }
    
    move_base_msgs::MoveBaseGoal waypoint;
    
    waypoint.target_pose.header.frame_id = "map";
    waypoint.target_pose.header.stamp = ros::Time::now();
    waypoint.target_pose.pose.position.x = trgt_X;
    waypoint.target_pose.pose.position.y = trgt_Y;
    waypoint.target_pose.pose.orientation.w = trgt_T;

    ac.sendGoal(waypoint);
    intransit = true;

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)        
	{
        ROS_INFO_STREAM("Success we made it to the target");
        intransit = false;		
        wait = false;
        ARG.data = true;
        pub.publish(ARG);
        ac.cancelAllGoals();
    }
	else
    {
        ROS_INFO_STREAM("Failure");
        intransit = false;
        wait = false;
        ARG.data = true;
        pub.publish(ARG);
        ac.cancelAllGoals();				
    }    
    ros::spin();
    
    return 0;
}

