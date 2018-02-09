#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher odom_pub;

void GoHome()
{
    ROS_INFO("Headed Home\n");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.41797494888;
    goal.target_pose.pose.position.y = -0.603062689304;
    goal.target_pose.pose.orientation.w = 1;

    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to known location");
    }
}

void GetPose()
{
}
void CommandCB(const std_msgs::String::ConstPtr &msg)
{

    if (msg->data == "Home")
    {
        GoHome();
    }
    else if (msg->data == "pose")
    {
        GetPose();
    }
}
void OdomCB(const nav_msgs::Odometry::ConstPtr &odom)
{
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tb_hub");
    ros::NodeHandle nh;

    ros::Subscriber home_sub;
    ros::Subscriber odom_sub;
    home_sub = nh.subscribe("tbCmd", 1, CommandCB);
    odom_sub = nh.subscribe("odom", 1000, OdomCB);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odomOut", 1000);

    ROS_INFO("tb_hub: Initiated. Waiting for new query.");

    ros::spin();

    return 0;
}