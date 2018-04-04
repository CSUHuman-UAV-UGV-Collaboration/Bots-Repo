#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "botsapp/TurtleStates.h"
#include "botsapp/ResourceString.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variable for updating and publishing the state
botsapp::TurtleStates _BotState;
ros::Publisher turtleState_pub;
ros::Publisher turtleResponse_pub;

void UpdateBotState(uint8_t botState)
{
    _BotState.BotState = botState;
    turtleState_pub.publish(_BotState);
}

void Search(float x, float y)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1;

    while (!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ac.sendGoal(goal);
    //Set the bot state to Moving
    UpdateBotState(botsapp::TurtleStates::MOVING);
    ac.waitForResult();

    //Set the bot state to Stationary
    UpdateBotState(botsapp::TurtleStates::STATIONARY);

    std_msgs::String msg;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to (%f,%f)", x, y);
        msg.data = "1";
        turtleResponse_pub.publish(msg);
    }
    else
    {
        ROS_INFO("Failed to navigate to (%f,%f)", x, y);
        msg.data = "0";
        turtleResponse_pub.publish(msg);
    }
}

void GoHome()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //TODO: Pass in (x,y)
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.41797494888;   // Hardcoded x-coord
    goal.target_pose.pose.position.y = -0.603062689304; // Hardcoded y-coord
    goal.target_pose.pose.orientation.w = 1;

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ac.sendGoal(goal);
    //Set the bot state to Moving
    UpdateBotState(botsapp::TurtleStates::MOVING);
    ac.waitForResult();

    //Set the bot state to Stationary
    UpdateBotState(botsapp::TurtleStates::STATIONARY);

    std_msgs::String msg;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to Home");
        msg.data = "1";
        turtleResponse_pub.publish(msg);
    }
    else
    {
        ROS_INFO("Failed to Navigate home");
        msg.data = "0";
        turtleResponse_pub.publish(msg);
    }
}

void TurtleCommandsCB(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("TurtleCommands init");
    string xString, yString;
    string cmdString;
    std::string::size_type sz;

    istringstream iss(msg->data.c_str());
    iss >> cmdString;
    iss >> xString;
    iss >> yString;

    float x = stof(xString);
    float y = stof(yString);

    if (cmdString == "search")
    {
        Search(x, y);
    }
    else if (cmdString == "home")
    {
        GoHome();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, botsapp::ResourceString::NODE_TURTLEHUB);
    ros::NodeHandle nh;
    ros::Rate loop_rate(1); // 1 message per second

    // Turtlebot Subscribers
    ros::Subscriber turtleRequest_sub = nh.subscribe(botsapp::ResourceString::TOPIC_TURTLEREQUEST, 1, TurtleCommandsCB);

    //Turtlebot Publishers
    turtleState_pub = nh.advertise<botsapp::TurtleStates>(botsapp::ResourceString::TOPIC_TURTLESTATE, 1);
    turtleResponse_pub = nh.advertise<std_msgs::String>(botsapp::ResourceString::TOPIC_TURTLERESPONSE, 1);

    //TODO: Add a function to do system checks
    UpdateBotState(botsapp::TurtleStates::STATIONARY);

    ROS_INFO("tb_hub: Initiated. Waiting for new query.");

    ros::spin();
    return 0;
}