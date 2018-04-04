#ifndef ROBOTCOLLAB_H // To make sure you don't declare the function more than once by including the header multiple times.
#define ROBOTCOLLAB_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <boost/lexical_cast.hpp>
#include "botsapp/TurtleStates.h"
#include "botsapp/DroneStates.h"
#include "botsapp/ResourceString.h"
#include "botsapp/Search.h"

using namespace std;

// define a class, including a constructor, member variables and member functions
class RobotCollab
{
  public:
    RobotCollab(ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    bool GoHome();
    bool Search(botsapp::Search);
    string GetBotStateAsString(uint8_t);

  private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    // Initializes robots Hubs
    string turtleResponse;
    string droneResponse;

    ros::Publisher turtleRequest_pub;

    ros::Subscriber turtleState_sub;
    ros::Subscriber turtleResponse_sub;

    //Initializes drone hub
    ros::Publisher droneRequest_pub;

    ros::Subscriber droneState_sub;
    ros::Subscriber droneResponse_sub;

    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    //State coordination
    uint8_t botState;
    uint8_t droneState;
    bool CanLand();
    bool Moveable();
    

    void BotStateCallback(const botsapp::TurtleStates &state);
    void BotResponseCallBack(const std_msgs::String::ConstPtr& msg);

    void DroneStateCallback(const botsapp::TurtleStates &state);
    void DroneResponseCallBack(const std_msgs::String::ConstPtr& msg);
};
#endif