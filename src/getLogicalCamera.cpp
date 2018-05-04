#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

double amcl_x;
double amcl_y;
double amcl_theta;

void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
   amcl_x = msg.pose.pose.position.x;
   amcl_y = msg.pose.pose.position.y;

   tf::Quaternion amclData;
   quaternionMsgToTF(msg.pose.pose.orientation, amclData);
   amcl_theta = getYaw(amclData);
}

void lcMessageReceived(const logical_camera_plugin::logicalImage &msg) {

	float x = ( msg.pose_pos_x * cos(amcl_theta) ) - ( msg.pose_pos_y * sin(amcl_theta) ) + amcl_x;
	float y = ( msg.pose_pos_x * sin(amcl_theta) ) + ( msg.pose_pos_y * cos(amcl_theta) ) + amcl_y;

	ROS_INFO_STREAM("modelName: " << msg.modelName);
	ROS_INFO_STREAM("x: " << x << ", y: " << y);
}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"getLogicalCamera");
	ros::NodeHandle nh;

	ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose",1000, &amclMessageReceived);
	ros::Subscriber subPose = nh.subscribe("/objectsDetected",1000, &lcMessageReceived);

	ros::spin();
}
