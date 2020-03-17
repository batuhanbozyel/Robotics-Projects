/*
 * explorer_node_a1_456.cpp
 *
 * BLG456E Assignment 1 skeleton
 *
 * Instructions: Change the laser_callback function to make the robot explore more intelligently, using its sensory data (the laser range array).
 *
 * Advanced: If you want to make use of the robot's mapping subsystem then you can make use of the map in the mapping_callback function.
 *
 * Name: Batuhan Bozyel
 *
 * Student ID: 150150031
 */

//Common ROS headers.
#include "ros/ros.h"

//This is needed for the data structure containing the motor command.
#include "geometry_msgs/Twist.h"
//This is needed for the data structure containing the laser scan.
#include "sensor_msgs/LaserScan.h"
//This is needed for the data structure containing the map (which you may not use).
#include "nav_msgs/OccupancyGrid.h"
//This is for easy printing to console.
#include <iostream>
#include <cmath>
#include <algorithm>

// For information on what publishing and subscribing is in ROS, look up the tutorials.
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
ros::Subscriber map_subscriber;

// For information on what a "message" is in ROS, look up the tutorials.
sensor_msgs::LaserScan laser_msg;
nav_msgs::OccupancyGrid map_msg;
geometry_msgs::Twist motor_command;


//The following function is a "callback" function that is called back whenever a new laser scan is available.
//That is, this function will be called for every new laser scan.
//
// --------------------------------------------------------------------------------------------
//CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY.
// --------------------------------------------------------------------------------------------
//
bool initial = true;
float previous_right = 10.0f;
float current_right = 10.0f;
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_msg=*msg;
    //data structure containing the command to drive the robot

    //Alternatively we could have looked at the laser scan BEFORE we made this decision.
    //Well, let's see how we might use a laser scan.
    std::vector<float> laser_ranges;
    laser_ranges=laser_msg.ranges;
    

    const float left = laser_ranges.size() - 1;
    const float right = 0;
    const float middle = laser_ranges.size() / 2;
    

    //Search for a wall
    bool isWall = false;
    for(int i = 0; i < laser_ranges.size() / 5; i++)
    {
        if(laser_ranges[middle + i] <= 0.75f || laser_ranges[middle - i] <= 0.75f)
        {
            isWall = true;
            break;
        }
    }

    previous_right = current_right;
    current_right = laser_ranges[right];
    //Search for a wall at initial state
    if(initial)
    {
        if(!isWall) 
        {
                motor_command.linear.x = 0.5f;
                motor_command.angular.z = 0.0f;
                std::cout << "Initial, No Walls" << std::endl;
        }
        else
        {
            if(laser_ranges[right] > 1.0f) 
            {
                motor_command.linear.x = 0.0f;
                motor_command.angular.z = 0.5f;
                std::cout << "Initial, Wall infront, Far from Right" << std::endl;
            }
            else
            {
                initial = false;
                std::cout << "Initial, Wall infront, Close to Right" << std::endl;
            }
        }
    }
    //Wall found, stick to it
    else
    {
        if((laser_ranges[right] <= 1.0f || std::isnan(laser_ranges[right])) && isWall)
        {
            std::cout << "Corner!" << std::endl;
            motor_command.linear.x = 0.0f;
            motor_command.angular.z = 0.7f;
        }
        else
        {
            float linear_factor = std::min(laser_ranges[middle - laser_ranges.size() / 5], laser_ranges[middle + laser_ranges.size() / 5]);
            if(std::isnan(linear_factor)) linear_factor = 10.0f;
            linear_factor = 0.5f - 0.5f/(0.5f + linear_factor);

            //Remain your distance relative to the right wall
            if(laser_ranges[right] >= 1.0f && laser_ranges[right] <= 1.1f)
            {
                motor_command.linear.x =  linear_factor;
                float angular_factor = 0.3f - 0.3f/(0.3f + laser_ranges[right]);
                if(current_right > previous_right) angular_factor = -angular_factor;
                motor_command.angular.z = angular_factor;
                std::cout << "Remain your distance!" << std::endl;
            }
            //Get away from the wall abit
            else if(laser_ranges[right] < 1.0f)
            {
                motor_command.linear.x =  linear_factor;
                float angular_factor = 2 * (0.6f - 0.6f/(0.6f + laser_ranges[right]));
                motor_command.angular.z = angular_factor;
                std::cout << "Get away from the wall abit!" << std::endl;
            }
            //Get closer to the wall
            else if(laser_ranges[right] > 1.1f)
            {
                motor_command.linear.x =  linear_factor;
                float angular_factor = -2 * (0.3f - 0.3f/(0.3f + laser_ranges[right]));
                motor_command.angular.z = angular_factor;
                std::cout << "Get closer to the wall!" << std::endl;
            }
            //Either too close to the wall, so laser scan can not measure
            //or too far away from the wall, so laser scan returns NaN value
            else
            {
                //On previous state, the robot could see a wall
                //So it just needs to turn right
                if(!std::isnan(previous_right))
                {
                    motor_command.linear.x =  linear_factor / 2;
                    motor_command.angular.z = -3 * linear_factor / 4;
                    std::cout << "NaN, turn right" << std::endl;
                }
                //If the robot couldn't see the wall on previous state
                //Then the robot is too close to the wall since it never attempts to leave the wall
                //So the robot needs to turn left abit, so it can see the wall again
                else
                {
                    motor_command.linear.x = 0.0f;
                    motor_command.angular.z = 0.3f;
                    std::cout << "NaN, turn left" << std::endl;
                }
                current_right = previous_right;
            }
        }
        
    }

    motor_command_publisher.publish(motor_command);
}
//
// --------------------------------------------------------------------------------------------
//
/*
//You can also make use of the map which is being built by the "gslam_mapping" subsystem
//There is some code here to help but you can understand the API also
// by looking up the OccupancyGrid message and its members (this is the API for the message).
//If you want me to explain the data structure I will - just ask me in advance of class.
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

    const bool chatty_map=false;

    map_msg=*msg;

    double map_width=map_msg.info.width;
    double map_height=map_msg.info.width;

    double map_origin_x = map_msg.info.origin.position.x;
    double map_origin_y = map_msg.info.origin.position.y;
    double map_orientation = acos(map_msg.info.origin.orientation.z);

    std::vector<signed char > map = map_msg.data;

    if(chatty_map)std::cout<<"------MAP:------"<<std::endl;
    // Here x and y are incremented by five to make the map fit in the terminal
    // Note that we have lost some map information by shrinking the data
    // this code is mostly to illustrate how the map grid data can be accessed
    for(unsigned int x=0; x<map_width; x+=5) {
        for(unsigned int y=0; y<map_height; y+=5) {

            unsigned int index = x + y*map_width;

            if(map[index] > 50) { // 0 – 100 represents how occupied
                //this square is occupied
                if(chatty_map)std::cout<<"X";
            } else if(map[index]>=0) {
                //this square is unoccupied
                if(chatty_map)std::cout<<" ";
            } else {
                //this square is unknown
                if(chatty_map)std::cout<<"?";
            }
        }
        if(chatty_map)std::cout<<std::endl;
    }
    if(chatty_map)std::cout<<"----------------"<<std::endl;
}
*/

int main(int argc, char **argv)
{
    // must always do this when starting a ROS node - and it should be the first thing to happen
    ros::init(argc, argv, "amble");
    // the NodeHandle object is our access point to ROS
    ros::NodeHandle n;

    // Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);

    // Here we set the function laser_callback to receive new laser messages when they arrive
    laser_subscriber = n.subscribe("/scan", 1000,laser_callback);
    // Here we set the function map_callback to receive new map messages when they arrive from the mapping subsystem
    //map_subscriber = n.subscribe("/map", 1000,map_callback);

    //now enter an infinite loop - the callback functions above will be called when new laser or map messages arrive.
    ros::Duration time_between_ros_wakeups(0.001);
    while(ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
    return 0;
}
