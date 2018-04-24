#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

void ekfMessageReceived(const nav_msgs::Odometry &msg) {
	ROS_INFO_STREAM("Recieved: " << msg.pose.covariance);

	Matrix<float, 3, 3> A(3, 3);

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			int pos = 6*i + j;
			if (msg.pose.covariance[pos] > 0) {
				A(i, j) = msg.pose.covariance[pos];
			}
		}
	}

	EigenSolver< Matrix<float, 3, 3> > es(A);

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
