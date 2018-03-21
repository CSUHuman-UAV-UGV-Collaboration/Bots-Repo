#ifndef ROBOTCOLLAB_H // To make sure you don't declare the function more than once by including the header multiple times.
#define ROBOTCOLLAB_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "botsapp/TurtleStates.h"
#include "botsapp/DroneStates.h"
#include "botsapp/ResourceString.h"
#include "botsapp/Search.h"
#include "botsapp/TurtleData.h"

using namespace std;

// define a class, including a constructor, member variables and member functions
class RobotCollab
{
  public:
    RobotCollab(ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    bool GoHome();
    bool Search(botsapp::Search);
    void PublishBotState(botsapp::TurtleStates);// TODO: this should probably be private
  string GetBotStateAsString(uint8_t);
  private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    // Initializes robots Hubs;
    ros::Publisher turtleState_pub;

    ros::Subscriber turtleState_sub;

    ros::ServiceClient GoHome_Serv;
    ros::ServiceClient Search_Serv;

    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    //State coordination
    uint8_t botState;
    uint8_t droneState;
    bool CanLand();
    bool Moveable();
    

    void BotStateCallback(const botsapp::TurtleStates &state);
};
#endif