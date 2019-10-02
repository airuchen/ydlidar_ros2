/*
 Copyright 2018 ADLINK Technology, Inc.
 Developer: 
  * Alan Chen (alan.chen@adlinktech.com
  * HaoChih, LIN (haochih.lin@adlinktech.com)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 This is the modified version of ydlidar pkg for ROS 2.0 environment.
 The original (ROS1) verion and sdk source code are belong to "EAI TEAM"
 (For further info: EAI official website: http://www.ydlidar.com)

*/
#define _USE_MATH_DEFINES
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)

rclcpp::Node::SharedPtr g_node = nullptr;

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int count = scan->scan_time / scan->time_increment;

    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) 
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	    if(degree > -5 && degree< 5)
        {
            printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("ydlidar_client");
    auto sub = g_node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1000, scanCallback);
    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
