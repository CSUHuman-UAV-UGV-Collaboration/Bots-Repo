// this header incorporates all the necessary #include files and defines the class "potentialField"
#include "potentialField.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
PotentialField::PotentialField(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    ROS_INFO("Potential Field Constructor");
    InitializeSubscribers();
    InitializePublishers();
    InitializeServices();

    //initialize variables and test here, as needed
}

//member helper function to set up subscribers;
// note odd syntax: &robotCollab::subscriberCallback is a pointer to a member function of robotCollab
// "this" keyword is required, to refer to the current instance of robotCollab
void PotentialField::InitializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

    // TODO initialize here
}

//member helper function to set up services:
void PotentialField::InitializeServices()
{
    ROS_INFO("Initializing Services");
}

//member helper function to set up publishers;
void PotentialField::InitializePublishers()
{
    ROS_INFO("Initializing Publishers");
    
    // TODO initialize here
}

// callbacks here
void PotentialField::Callback()
{
    
}


pixel PotentialField::GetOpenSpace(pixel robotLocation)
{
    float potential;
    // call get obstacles
    // min = inf;
    // pixel minPos;
    // for each pixel in range r
    //      potential = 0;
    //      for all obstacles in PotentialField::obstacles
    //          potential = vector add RepulsivePotential(point, obstacle)
    //      if potential < min
    //          min_pos = pixel
}

float PotentialField::RepulsivePotential(pixel point, pixel obstacle)
{

}
