#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

bool searching = 1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "search");
    ros::NodeHandle nh;

    // This is the first possible idea of trying
    //  to create a way of determining the actions
    //  of the robot
    //
    // Possible improvements: 
    //  Have a state-machine to control robot behavior
    //  "Higher Level" node that will pass arguments to 
    //      the state machine. Could lose efficiency
    while(searching)
    {
        // do something to search the map
    }
}
