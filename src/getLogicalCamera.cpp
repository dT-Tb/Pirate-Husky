#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>

void lcMessageReceived(const logical_camera_plugin::logicalImage &msg) {
/*
	msg.modelName;
	msg.pose_pos_x;
	msg.pose_pos_y;
	msg.pose_pos_z;
	msg.pose_rot_x;
	msg.pose_rot_y;
	msg.pose_rot_z;
	msg.pose_rot_w;
*/

ROS_INFO_STREAM("modelName: " << msg.modelName);
ROS_INFO_STREAM("pose_pos_x: " << msg.pose_pos_x);
ROS_INFO_STREAM("pose_pos_y: " << msg.pose_pos_y);
ROS_INFO_STREAM("pose_pos_z: " << msg.pose_pos_z);
ROS_INFO_STREAM("pose_rot_x: " << msg.pose_rot_x);
ROS_INFO_STREAM("pose_rot_y: " << msg.pose_rot_y);
ROS_INFO_STREAM("pose_rot_z: " << msg.pose_rot_z);
ROS_INFO_STREAM("pose_rot_w: " << msg.pose_rot_w);

}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"getLogicalCamera");
	ros::NodeHandle nh;

	ros::Subscriber subPose = nh.subscribe("/objectsDetected",1000, &lcMessageReceived);

	ros::spin();
	return 0;
}
