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
bool wait = false;

double trgt_X, trgt_Y, trgt_T;

void gotToPose(const geometry_msgs::Pose2D msg)
{
    trgt_X = msg.x;
    trgt_Y = msg.y;
    trgt_T = msg.theta;
    ROS_INFO_STREAM("Sub x:"<< trgt_X);
    ROS_INFO_STREAM("Sub y:"<< trgt_Y);
    ROS_INFO_STREAM("Sub t:"<< trgt_T);
    wait = true;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "huskyExplore");
    ros::NodeHandle nh;

    ros::Subscriber pointSub = nh.subscribe("Waypoints", 1000, gotToPose);
    
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/MORE", 1000);
    
    ros::Publisher stop = nh.advertise<std_msgs::Bool>("NoMore", 1000);
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
    ROS_INFO_STREAM("Waiting on server my dude...");

    // ros::Subscriber sub = nh.subscribe("/scan", 1000, laserFeed);

    while (!ac.waitForServer()){/*waiting for server to connect*/}  
    ROS_INFO_STREAM("Waiting on points");
    ros::Rate rate(15);


    while(ros::ok())
    { 
        ros::spinOnce();
        if(wait)
        {
            ROS_INFO_STREAM("Moving to Dest");
            
            std_msgs::Bool Stp;
            
            Stp.data = true;
            stop.publish(Stp);

            move_base_msgs::MoveBaseGoal waypoint;
            
            waypoint.target_pose.header.frame_id = "map";
            waypoint.target_pose.header.stamp = ros::Time::now();
            waypoint.target_pose.pose.position.x = trgt_X;
            waypoint.target_pose.pose.position.y = trgt_Y;
            waypoint.target_pose.pose.orientation.w = trgt_T;

            ac.sendGoal(waypoint);
            intransit = true;
            std_msgs::Bool ARG;

            ac.waitForResult();
            // ros::spinOnce();  
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)        
            {
                ROS_INFO_STREAM("Success we made it to the target");
                intransit = false;		
                wait = false;
                ARG.data = true;
                ac.cancelAllGoals();
                ROS_INFO_STREAM("Requesting new points");
                pub.publish(ARG);
            }
            else
            {
                ROS_INFO_STREAM("Better luck next time");
                intransit = false;
                wait = false;
                ARG.data = true;
                ac.cancelAllGoals();				
                pub.publish(ARG);
                ROS_INFO_STREAM("Requesting new points");
            }  
            // ros::spinOnce();  
        }
        ros::spinOnce();
    }
    ros::spin();
    
    return 0;
}

