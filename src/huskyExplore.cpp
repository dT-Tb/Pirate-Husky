#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cstdlib>


#define Pi 3.1415926

bool intransit = false;
bool wait = false;
bool avoid = false;

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
void DoNothing(const std_msgs::Bool no)
{
    if(no.data == true)
    {
        avoid = true;
    }
    if(no.data == false)
    {
        avoid = false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "huskyExplore");
    ros::NodeHandle nh;

    ros::Subscriber pointSub = nh.subscribe("Waypoints", 1000, gotToPose);
    ros::Subscriber avoidance = nh.subscribe("/obstacleDetected", 1000, DoNothing);
    
    ros::Publisher Intransit = nh.advertise<std_msgs::Bool>("/goalStatus",1000);
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/MORE", 1000);
    ros::Publisher stop = nh.advertise<std_msgs::Bool>("NoMore", 1000);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
    ROS_INFO_STREAM("Waiting on server my dude...");


    while (!ac.waitForServer()){/*waiting for server to connect*/}  
    ROS_INFO_STREAM("Waiting on points");
    ros::Rate rate(15);


    while(ros::ok())
    { 
        ros::spinOnce();
        if(wait && !avoid)
        {
            ROS_INFO_STREAM("Moving to Dest");
            
            std_msgs::Bool Stp;
            std_msgs::Bool inform;
            
            inform.data = true;
            Intransit.publish(inform);

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
                inform.data = false;
                Intransit.publish(inform);
            }
            else if(ac.waitForResult(ros::Duration(5.0)))
            {
                ROS_INFO_STREAM("Robot Stuck? Going to Search portocol!!");
                intransit = false;
                wait = false;
                ac.cancelAllGoals();				
                inform.data = false;
                Intransit.publish(inform);
            }
            else

                ROS_INFO_STREAM("Better luck next time");
                intransit = false;
                wait = false;
                ARG.data = true;
                ac.cancelAllGoals();				
                pub.publish(ARG);
                ROS_INFO_STREAM("Requesting new points");
                inform.data = false;
                Intransit.publish(inform);
            }  
        }
        if(avoid)
        { // do nothing until told so 
            std_msgs::Bool info;
            info.data = false;
            Intransit.publish(info);
            ros::spinOnce();
        }
    }// end while
    
}

