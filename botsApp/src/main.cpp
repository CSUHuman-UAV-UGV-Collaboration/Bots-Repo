#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher TBhub_pub;
ros::Subscriber TBpose_sub;

void OdomCB(const nav_msgs::Odometry::ConstPtr &odom)
{
    cout << "odom x: " << odom->pose.pose.position.x;
    cout << "odom y: " << odom->pose.pose.position.y;
    cout << "odom z: " << odom->pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main");
    ros::NodeHandle n;

    std_msgs::String msg;

    TBhub_pub = n.advertise<std_msgs::String>("tbCmd", 1000);
    TBpose_sub = n.subscribe("odomOut", 1, OdomCB);

    int searchCode;
    do
    {
        cout << "--------------------------------------\n";
        cout << "What would you like me to find?\n";
        cout << "0 - Go Home\n";
        cout << "-1 - to Quit\n";
        cout << "--------------------------------------\n";
        cout << "Enter Value: ";
        cin >> searchCode;

        switch (searchCode)
        {
        case 0:

            cout << "Command sent to go home\n";
            msg.data = "Home";
            TBhub_pub.publish(msg);
            break;

        case -1:

            cout << "Have a nice day!\n";
            break;

        default:

            cout << "Item not found\n";
            break;
        }
    } while (searchCode != -1);

    return 0;
}
