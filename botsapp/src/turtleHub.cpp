#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "robotCollab.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variable for updating and publishing the state
botsapp::TurtleStates _BotState;

botsapp::TurtleStates GetBotState()
{
    return _BotState;
}

void SetBotState(const botsapp::TurtleStates &botState)
{
    _BotState = botState;
    ROS_INFO("myCallback activated: received value %d", _BotState.BotState);
}

bool Search(botsapp::TurtleDataRequest &request, botsapp::TurtleDataResponse &response)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //TODO: Pass in (x,y)
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = request.x;
    goal.target_pose.pose.position.y = request.y;
    goal.target_pose.pose.orientation.w = 1;

    while (!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ac.sendGoal(goal);
    //Set the bot state to Moving
    _BotState.BotState = botsapp::TurtleStates::MOVING;
    ac.waitForResult();

    //Set the bot state to Stationary
    _BotState.BotState = botsapp::TurtleStates::STATIONARY;

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to (%f,%f)",request.x,request.y);
        return true;
    }
    else
    {
        ROS_INFO("Failed to navigate to (%f,%f)",request.x,request.y);
        return false;
    }
}

bool GoHome(botsapp::TurtleDataRequest &request, botsapp::TurtleDataResponse &response)
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
    _BotState.BotState = botsapp::TurtleStates::MOVING;
    ac.waitForResult();

    //Set the bot state to Stationary
    _BotState.BotState = botsapp::TurtleStates::STATIONARY;

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to Home");
        return true;
    }
    else
    {
        ROS_INFO("Failed to Navigate home");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, botsapp::ResourceString::NODE_TURTLEHUB);
    ros::NodeHandle nh;
    ros::Rate loop_rate(1); // 1 message per second

    //TODO: Add a function to do system checks
    _BotState.BotState = botsapp::TurtleStates::STATIONARY;

    // Turtlebot Subscribers
    ros::Subscriber turtleState_sub = nh.subscribe(botsapp::ResourceString::TOPIC_TURTLESTATE, 1, SetBotState);

    //Turtlebot Publishers
    ros::Publisher turtleState_pub = nh.advertise<botsapp::TurtleStates>(botsapp::ResourceString::TOPIC_TURTLESTATE, 1);

    //Turtlebot Services
    ros::ServiceServer goHome_serv = nh.advertiseService(botsapp::ResourceString::SERV_GOHOME, GoHome);
    ros::ServiceServer search_serv = nh.advertiseService(botsapp::ResourceString::SERV_SEARCH, Search);

    ROS_INFO("tb_hub: Initiated. Waiting for new query.");

    while (ros::ok())
    {
        turtleState_pub.publish(GetBotState());

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}