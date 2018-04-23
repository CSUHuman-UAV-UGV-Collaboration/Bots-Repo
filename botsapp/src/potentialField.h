#ifndef POTENTIALFIELD_H // To make sure you don't declare the function more than once by including the header multiple times.
#define POTENTIALFIELD_H

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

struct pixel
{
    float x;
    float y;
};

// define a class, including a constructor, member variables and member functions
class PotentialField
{
  public:
    PotentialField(ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    pixel GetOpenSpace(pixel robotLocation);

  private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    //list of obstacles
    vector<pixel> obstacles;

    void GetObstacles();
    float RepulsivePotential(pixel point, pixel obstacle);
};
#endif