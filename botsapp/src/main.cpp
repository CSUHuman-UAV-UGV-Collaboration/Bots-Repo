#include "robotCollab.h"
#include "potentialField.h"

int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "robotCollab"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type RobotCollab");
    RobotCollab robotCollab(&nh);

    // potential field TEST
    geometry_msgs::PoseWithCovariance pose;

    pose.pose.position.x = 2.0;
    pose.pose.position.y = -2.4;
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;
    if (client.call(srv))
    {
        Pixel pixel;
        pixel.x = 2;
        pixel.y = 1;
        nav_msgs::OccupancyGrid grid = srv.response.map;

        PotentialField potentialField(&nh);
        potentialField.GetSpaceInRange(pose, grid);
        Pixel freePose = potentialField.GetFreePose();

        cout << freePose.x;
        cout << freePose.y;
        cout << "\n";
    }

    bool completed;
    botsapp::Search searchMsg;
    int searchCode;
    do
    {
        //Tests-------------------------------------------------
        botsapp::TurtleStates state;

        //robotCollab.Search(state);
        // state.BotState = botsapp::States::MOVING;
        //------------------------------------------------------------
        cout << "--------------------------------------\n";
        cout << "What would you like me to do?\n";
        cout << "1 - Go Home\n";
        cout << "2 - Search\n";
        cout << "-1 - to Quit\n";
        cout << "--------------------------------------\n";
        cout << "Enter Value: ";
        cin >> searchCode;

        switch (searchCode)
        {
        case 1:
            cout << "Command sent to go home\n";
            //call go home
            completed = robotCollab.GoHome();
            break;
        case 2:
            cout << "Command to search\n";
            cout << "Enter x coord";
            cin >> searchMsg.x;
            cout << "Enter y coord";
            cin >> searchMsg.y;
            searchMsg.useExternal = false;
            completed = robotCollab.Search(searchMsg);
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
