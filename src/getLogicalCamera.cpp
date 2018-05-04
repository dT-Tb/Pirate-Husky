#include <ros/ros.h>
// #include <logical_camera_plugin/logicalImage.h>
// #include <geometry_msgs/Pose.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

// void lcMessageReceived(const logical_camera_plugin::logicalImage &msg) {
/*
	msg.pose_pos_x;
	msg.pose_pos_y;
	msg.pose_pos_z;
	msg.pose_rot_x;
	msg.pose_rot_y;
	msg.pose_rot_z;
	msg.pose_rot_w;
*/
	// tf::TransformBroadcaster br;
	// tf::TransformListener listener;
	// tf::Transform transform1;
	// tf::StampedTransform transform2;

	// transform1.setOrigin( tf::Vector3(msg.pose_pos_x, msg.pose_pos_y, msg.pose_pos_z) );
	// transform1.setRotation( tf::Quaternion(msg.pose_rot_x, msg.pose_rot_y, msg.pose_rot_z, msg.pose_rot_z) );
	// br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", msg.modelName));

	// listener.lookupTransform("map", msg.modelName, ros::Time(0), transform2);

	// ROS_INFO_STREAM("modelName: " << msg.modelName);
	// ROS_INFO_STREAM("msg.pose_pos_x: " << transform2.getOrigin().x());
	// ROS_INFO_STREAM("msg.pose_pos_y: " << transform2.getOrigin().y());
	// ROS_INFO_STREAM("msg.pose_pos_z: " << transform2.getOrigin().z());
	// ROS_INFO_STREAM("msg.pose_rot_x: " << transform2.getRotation().x());
	// ROS_INFO_STREAM("msg.pose_rot_y: " << transform2.getRotation().y());
	// ROS_INFO_STREAM("msg.pose_rot_z: " << transform2.getRotation().z());
	// ROS_INFO_STREAM("msg.pose_rot_w: " << transform2.getRotation().w());
// }

int main(int argc,char **argv) 
{
	// ros::init(argc,argv,"getLogicalCamera");
	// ros::NodeHandle nh;

	// ros::Subscriber subPose = nh.subscribe("/objectsDetected",1000, &lcMessageReceived);

	// ros::spin();
	return 0;
}
