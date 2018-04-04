// this header incorporates all the necessary #include files and defines the class "robotCollab"
#include "robotCollab.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
RobotCollab::RobotCollab(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of RobotCollab");
    InitializeSubscribers();
    InitializePublishers();
    InitializeServices();

    //initialize variables and test here, as needed

    //TODO Check if Robot hubs are alive

    // ROS_INFO_STREAM("Stationary = " << botsapp::States::STATIONARY);
    // ROS_INFO_STREAM("MOVING = " << botsapp::States::MOVING);
}

//member helper function to set up subscribers;
// note odd syntax: &robotCollab::subscriberCallback is a pointer to a member function of robotCollab
// "this" keyword is required, to refer to the current instance of robotCollab
void RobotCollab::InitializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    turtleState_sub = nh_.subscribe(botsapp::ResourceString::TOPIC_TURTLESTATE, 1, &RobotCollab::BotStateCallback, this);
    turtleResponse_sub = nh_.subscribe(botsapp::ResourceString::TOPIC_TURTLERESPONSE, 1, &RobotCollab::BotResponseCallBack, this);

    droneState_sub = nh_.subscribe(botsapp::ResourceString::TOPIC_DRONESTATE, 1, &RobotCollab::DroneStateCallback, this);
    droneResponse_sub = nh_.subscribe(botsapp::ResourceString::TOPIC_DRONERESPONSE, 1, &RobotCollab::DroneResponseCallBack, this);
}

//member helper function to set up services:
void RobotCollab::InitializeServices()
{
    ROS_INFO("Initializing Services");
}

//member helper function to set up publishers;
void RobotCollab::InitializePublishers()
{
    ROS_INFO("Initializing Publishers");
    turtleRequest_pub = nh_.advertise<std_msgs::String>(botsapp::ResourceString::TOPIC_TURTLEREQUEST, 2);
    droneRequest_pub = nh_.advertise<std_msgs::String>(botsapp::ResourceString::TOPIC_DRONEREQUEST, 2);
}

void RobotCollab::BotStateCallback(const botsapp::TurtleStates &state)
{
    botState = state.BotState;
    //ROS_INFO("myCallback activated: received value %d", botState);
}

void RobotCollab::BotResponseCallBack(const std_msgs::String::ConstPtr &msg)
{
    turtleResponse = msg->data;
}

void RobotCollab::DroneStateCallback(const botsapp::TurtleStates &state)
{
    botState = state.BotState;
    //ROS_INFO("myCallback activated: received value %d", botState);
}

void RobotCollab::DroneResponseCallBack(const std_msgs::String::ConstPtr &msg)
{
    droneResponse = msg->data;
}

bool RobotCollab::GoHome()
{
    ros::spinOnce();

    //check if robots are "Moveable"
    // TODO: Add a timeout
    while (Moveable() != true)
    {
        ROS_INFO("Transitioning States");
        ROS_INFO_STREAM("Bot state: " << GetBotStateAsString(botState)); // << | Drone state: %d", , droneState);

        //TODO: code here for getting it into a moveable state.
        ros::spinOnce();
        ros::Duration(1.5).sleep(); // sleep for 1.5 second
    }

    ROS_INFO("Headed Home");
    //TODO: snd request to go home
    // if (GoHome_Serv.call(srv))
    // {
    return true;
    // }

    ROS_INFO("Something went horribly wrong in GoHome function");
    return false;

    // std_msgs::String msg;

    // msg.data = "Home";
    // turtleHub_pub.publish(msg);
    // cout << "Go Home initiated...";

    // int scanCounter = 1;
    // ros::Time start_time = ros::Time::now();
    // ros::Rate rate(1);
    // while (ros::Time::now() - start_time < ros::Duration(60.0))
    // {
    //     ROS_INFO("Going home in progress. %d", scanCounter);
    //     // wait for a subscriber to modify a value and break this when value is true.
    //     scanCounter++;

    //     ros::spinOnce();
    //     rate.sleep();
    // }
}

bool RobotCollab::Search(botsapp::Search searchMsg)
{
    turtleResponse = "0";
    droneResponse = "0";

    if (searchMsg.useExternal == true)
    {
        //TODO: do something here to wait for rviz goal or some other external goal
    }
    else
    {
        /*
        Checks if the the robot state is moveable and moves to the destinantion.
         keeps listening for turtleResponse to update before moving to next step. */

        //TODO: fix duplicates by moving this in one function
        while (Moveable() != true)
        {
            ROS_INFO("Transitioning States");
            ROS_INFO_STREAM("Bot state: " << GetBotStateAsString(botState));
            //   ROS_INFO_STREAM("Drone state: " << GetBotStateAsString(botState));

            //TODO: code here for getting it into a moveable state.
            ros::spinOnce();
            ros::Duration(1.5).sleep(); // sleep for 1.5 second
        }
        std_msgs::String msg;
        msg.data = "search " +
                   boost::lexical_cast<std::string>(searchMsg.x) +
                   " " +
                   boost::lexical_cast<std::string>(searchMsg.y);
        turtleRequest_pub.publish(msg);

        while (turtleResponse == "0")
        {
            ros::spinOnce();
            ros::Duration(1.5).sleep(); // sleep for 1.5 second
        }
        turtleResponse = "0";

        /*
        Checks if the the drone state to assure it can fly and takes off.
         keeps listening for droneResponse to update before moving to next step. */

        // TODO: check state of drone and make sure its safe to take off
        msg.data = "takeoff";
        droneRequest_pub.publish(msg);

        while (droneResponse == "0")
        {
            ros::spinOnce();
            ros::Duration(1.5).sleep(); // sleep for 1.5 second
        }
        droneResponse = "0";

        /*
        Checks if the the drone state to assure it can lan and land the drone.
         keeps listening for droneResponse to update before moving to next step. */

        // TODO: check state of drone and make sure its safe to Land
        msg.data = "land";
        droneRequest_pub.publish(msg);
        while (droneResponse == "0")
        {
            ros::spinOnce();
            ros::Duration(1.5).sleep(); // sleep for 1.5 second
        }
        return true;
    }
    // TODO: Return false if something goes wrong..
}

bool RobotCollab::Moveable()
{
    //TODO: what if robot is given goal outside of this app check robot system for movements?
    if (droneState == botsapp::DroneStates::DOCKED && botState == botsapp::TurtleStates::STATIONARY)
        return true;
    //else
    //TODO: get bot or drone to right state
}

bool RobotCollab::CanLand()
{
    if (droneState == botsapp::DroneStates::FLYING && botState == botsapp::TurtleStates::STATIONARY)
        return true;
    else
        return false;
}

string RobotCollab::GetBotStateAsString(uint8_t state)
{
    switch (state)
    {
    case 0:
        return botsapp::ResourceString::STATE_STATIONARY;
    case 1:
        return botsapp::ResourceString::STATE_MOVING;
    default:
        return "";
    }
}
