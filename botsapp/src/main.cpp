#include "robotCollab.h"

int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "robotCollab"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type RobotCollab");
    RobotCollab robotCollab(&nh);

    int searchCode;
    do
    {
        bool completed;
        botsapp::Search searchMsg;

        //TODO: add resource string for hard coded values

        //Tests-------------------------------------------------
        botsapp::States state;

        //robotCollab.Search(state);
        // state.BotState = botsapp::States::MOVING;
        // robotCollab.PublishBotState(state);
        //_------------------------------------------------------------
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
            searchMsg.useExternal = true;
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
