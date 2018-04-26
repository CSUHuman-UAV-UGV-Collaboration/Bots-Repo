// this header incorporates all the necessary #include files and defines the class "potentialField"
#include "potentialField.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
PotentialField::PotentialField(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    ROS_INFO("Potential Field Constructor");
    //initialize variables and test here, as needed
}

void PotentialField::GetSpaceInRange(geometry_msgs::PoseWithCovariance robotPose, nav_msgs::OccupancyGrid map)
{
    int map_height, map_width;
    Pixel robotLocation;

    map_height = map.info.height;
    map_width = map.info.width;
    map_resolution = map.info.resolution;
    map_origin = map.info.origin;

    // Parse map into int[]
    int oneDimenGrid[map_height * map_width];
    for (int x = 0; x < (map_height * map_width); x++)
    {
        oneDimenGrid[x] = map.data[x];
    }

    //Convert map into two dimensional vector
    std::vector<std::vector<Pixel>> grid = ConvertTo2DArray(map_height, map_width, oneDimenGrid);

    //get robot location relative to the map
    robotLocation.x = (robotPose.pose.position.x - map_origin.position.x) / map_resolution;
    robotLocation.y = (robotPose.pose.position.y - map_origin.position.y) / map_resolution;
    std::cout << "Robot x coord: " << robotLocation.x;
    std::cout << "Robot y coord: " << robotLocation.y;
    std::cout << "\n";

    // get all spaces in range

    int top = std::max(0, (int)robotLocation.y - RANGE);
    int bottom = std::min((int)robotLocation.y + RANGE, map_height);
    int left = std::max(0, (int)robotLocation.x - RANGE);
    int right = std::min((int)robotLocation.x + RANGE, map_width);

    std::cout << "Top: " << top;
    std::cout << "bottom: " << bottom;
    std::cout << "left: " << left;
    std::cout << "right: " << right;
    std::cout << "\n";

    for (int i = left; i <= right; i++)
    {
        for (int j = top; j <= bottom; j++)
        {
            Pixel newPix;
            newPix.x = i;
            newPix.y = j;
            newPix.occupancyValue = grid[i][j].occupancyValue;
            allSpaces.push_back(newPix);
            // std::cout << newPix.x << newPix.y << " ";
            std::cout << newPix.occupancyValue << " ";
        }
        std::cout << "\n";
    }

    for (int space = 0; space < allSpaces.size(); space++)
    {
        if (allSpaces[space].occupancyValue == 100)
            obstacles.push_back(allSpaces[space]);
        else if (allSpaces[space].occupancyValue == 0)
            frees.push_back(allSpaces[space]);
    }

    std::cout << "Total Spaces: " << allSpaces.size() << std::endl;
    std::cout << "Obstacles Spaces: " << obstacles.size() << std::endl;
    std::cout << "Free Space: " << frees.size() << std::endl;

    //Trinary
    // The standard interpretation is the trinary interpretation, i.e. interpret all values so that the output ends up being one of three values.

    // If p > occupied_thresh, output the value 100 to indicate the cell is occupied.

    // If p < free_thresh, output the value 0 to indicate the cell is free.

    // Otherwise, output -1 a.k.a. 255 (as an unsigned char), to indicate that the cell is unknown.
}

std::vector<std::vector<Pixel>> PotentialField::ConvertTo2DArray(int height, int width, int data[])
{
    std::vector<std::vector<Pixel>> tempGrid(height, std::vector<Pixel>(width));
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            tempGrid[i][j].occupancyValue = (int)data[i + j * height];
            tempGrid[i][j].x = i;
            tempGrid[i][j].y = j;
        }
    }
    return tempGrid;
}

Potential PotentialField::RepulsivePotential(Pixel pixel, Pixel obstacle)
{

    // Euclidean distance rho
    float x = obstacle.x - pixel.x;
    float y = obstacle.y - pixel.y;
    float rho;
    Potential u;
    float direction;
    float magnitude;

    rho = std::pow(x, 2) + std::pow(y, 2);
    rho = std::sqrt(rho);

    direction = atan2(y, x);

    if (rho < RHO_0)
        magnitude = 0.5 * ETA * std::pow((1 / rho - 1 / RHO_0), 2);
    else
        magnitude = 0;

    u.x = magnitude * cos(direction * PI / 180.0);
    u.y = magnitude * sin(direction * PI / 180.0);

    return u;
}

Pixel PotentialField::GetFreePose()
{
    Pixel current;
    Pixel obstacle;
    Potential potential;
    Potential currentPotential;
    float min = 99999;
    Pixel minPixel;
    float mag = 0;

    for (int i = 0; i < frees.size(); i++)
    {
        current = frees[i];
        for (int j = 0; j < obstacles.size(); j++)
        {
            obstacle = obstacles[j];
            currentPotential = RepulsivePotential(current, obstacle);
            potential.x += currentPotential.x;
            potential.y += currentPotential.y;
        }

        mag = sqrt(pow(potential.x, 2) + pow(potential.y, 2));

        if (mag < min)
        {
            min = mag;
            minPixel.x = current.x;
            minPixel.y = current.y;
        }
    }
    Pixel robotLocation;
    robotLocation.x = (minPixel.x * map_resolution) + map_origin.position.x;
    robotLocation.y = (minPixel.y * map_resolution) + map_origin.position.y;
    return robotLocation;
}
