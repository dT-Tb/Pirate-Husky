#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Eigenvalues>

using namespace Eigen;
using namespace std;

void ekfMessageReceived(const nav_msgs::Odometry &msg) {
	//ROS_INFO_STREAM("Recieved: " << msg.pose.covariance);

	Matrix3f A(3, 3);

	//these indexs of cov represent the x, y, and yaw (1, 2, 6 of 6x6 matrix)
	A(0, 0) = msg.pose.covariance[0];
	A(0, 1) = msg.pose.covariance[1];
	A(0, 2) = msg.pose.covariance[5];
	A(1, 0) = msg.pose.covariance[6];
	A(1, 1) = msg.pose.covariance[7];
	A(1, 2) = msg.pose.covariance[11];
	A(2, 0) = msg.pose.covariance[30];
	A(2, 1) = msg.pose.covariance[31];
	A(2, 2) = msg.pose.covariance[35];

	EigenSolver< Matrix3f > es(A);

	complex<double> lambda1 = es.eigenvalues()[0];
	complex<double> lambda2 = es.eigenvalues()[1];
	complex<double> lambda3 = es.eigenvalues()[2];

	float chi = 9.348;
	float pi = 3.14159;

	float a = sqrt( lambda1.real() * chi );
	float b = sqrt( lambda2.real() * chi );
	float c = sqrt( lambda3.real() * chi );

	float volume = 4/3 * a * b * c * pi;

	ROS_INFO_STREAM("Volume: " << volume);
}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"uncertainty");
	ros::NodeHandle nh;

	ros::Subscriber subPose = nh.subscribe("/odometry/filtered",1000, &ekfMessageReceived);

	ros::spin();
	return 0;
}
