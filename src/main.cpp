#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>

using namespace std;

ros::Publisher TBhub_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main");
    ros::NodeHandle n;

    std_msgs::String msg;

    TBhub_pub = n.advertise<std_msgs::String>("tbCmd", 1000);

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
        {
            cout << "Headed Home\n";
             msg.data = "Home";
            TBhub_pub.publish(msg);
            
            break;
        }
        case -1:
        {
            cout << "Have a nice day!\n";
            break;
        }
        default:
        {
            cout << "Item not found\n";
            break;
        }
        }
    } while (searchCode != -1);

    return 0;
}
