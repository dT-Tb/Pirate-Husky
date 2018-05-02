#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>

int main(int argc, char **argv){

  ros::init(argc, argv, "huskyPose");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  ros::Rate rate(1.0);

  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
        ROS_INFO("Transformation Update!: x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("ERROR: %s", ex.what());
    } 
    rate.sleep();
  }
  ROS_ERROR("404 Error lol ;)");
}
