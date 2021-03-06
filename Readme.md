# Bots-Repo - UAV-UGV collaboration

## Cleveland State University (Senior design)
=============================================
## Team Members
  * Sujay Bajracharya
  * Austin Cassill
  * Lakiel Wade
  * William Heeter

## Packages required
* we can write stuff here
  * this is a list
  * list item
  
## TODO
  * Astra camera: get the launch file working.
    * we can look at the small laptops and the email that shiqi sent recently
  * Simulation/Gazebo: get the robots fully controllable.
    * see in the launch file for the joystick teleop for rotors.
    
## Gazebo Simulator 
  With the Catkin_ws setup run 
   * roslaunch rotors_gazebo bebop_hover.launch
  
  to control the turtlebot run
   * roslaunch turtlebot_teleop  keyboard_teleop.launch
   
  to control the drone 
  * Coming Soon

## Network Setup
Instructions to setup the network to control the bots from a remote computer.
NOTE: must be on same network

### On master PC
  run these cmds
 * roscore
 * hostname -I
 * export ROS_IP="yourip"

### On slave PC
  run these cmds
 * export ROS_MASTER_URI=http//"masterip":11311
 * hostname -I
 * export ROS_IP="yourip"

## ORB_SLAM2
Github:
 * https://github.com/raulmur/ORB_SLAM2

Run command: 
 * rosrun ORB_SLAM2 Mono [Vocabulary file] [settings file] /camera/image_raw:=/bebop/image_raw
 * Example: rosrun ORB_SLAM2 Mono ./Vocabulary/ORBvoc.txt ~/Bots-Repo/Camera/bebop2_cam.yaml /camera/image_raw:=/bebop/image_raw

Now we need a way save and load map

Issues:
 * We need to be able to save and load maps

## Addtional Dependencies
 * https://github.com/AutonomyLab/bebop_autonomy
 * https://github.com/ethz-asl/mav_comm
 * https://github.com/ros-teleop/teleop_twist_keyboard
 * https://github.com/AutonomyLab/parrot_arsdk
 * https://github.com/CSUHuman-UAV-UGV-Collaboration/ar_track_alvar.git (Use Sudo apt-get ros-kinetic-ar-track-alvar)
