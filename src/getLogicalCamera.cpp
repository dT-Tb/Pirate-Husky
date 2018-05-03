#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform.h>

void lcMessageReceived(const logical_camera_plugin::logicalImage &msg) {
/*
	msg.pose_pos_x;
	msg.pose_pos_y;
	msg.pose_pos_z;
	msg.pose_rot_x;
	msg.pose_rot_y;
	msg.pose_rot_z;
	msg.pose_rot_w;
*/
	tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(msg.pose_pos_x, msg.pose_pos_y, msg.pose_pos_z) );
	transform.setRotation( tf::Quaternion(msg.pose_rot_x, msg.pose_rot_y, msg.pose_rot_z, msg.pose_rot_z) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", msg.modelName));

	listener.lookupTransform("map", msg.modelName, ros::Time(0), transform);

	ROS_INFO_STREAM("modelName: " << msg.modelName);
	ROS_INFO_STREAM("msg.pose_pos_x: " << transform.getOrigin().x());
	ROS_INFO_STREAM("msg.pose_pos_y: " << transform.getOrigin().y());
	ROS_INFO_STREAM("msg.pose_pos_z: " << transform.getOrigin().z());
	ROS_INFO_STREAM("msg.pose_rot_x: " << transform.getRotation().x());
	ROS_INFO_STREAM("msg.pose_rot_y: " << transform.getRotation().y());
	ROS_INFO_STREAM("msg.pose_rot_z: " << transform.getRotation().z());
	ROS_INFO_STREAM("msg.pose_rot_w: " << transform.getRotation().w());
}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"getLogicalCamera");
	ros::NodeHandle nh;

	ros::Subscriber subPose = nh.subscribe("/objectsDetected",1000, &lcMessageReceived);

	ros::spin();
	return 0;
}
