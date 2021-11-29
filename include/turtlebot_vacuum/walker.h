/**
 * Copyright (C) MIT.
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @copyright  MIT License (c) 2021 Pratik Acharya
 * @file  walker.h
 * @brief A header for walker.cpp
 * @author Pratik Acharya
 */
#ifndef ROOMBA_INCLUDE_TURTLEBOT_VACUUM_WALKER_H_
#define ROOMBA_INCLUDE_TURTLEBOT_VACUUM_WALKER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**
 * @class TurtlebotNav
 * @brief A class to define methods and attributes for ROS node
 *
 */
class TurtlebotNav {
 private:
  // To store the twist function
  geometry_msgs::Twist twist;
  // NodeHandle object
  ros::NodeHandle nh;

 public:

  // Publishing the navigation data to cmd_vel topic
  ros::Publisher pub = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 10);
  // Subscribing to the lidar scan topic
  ros::Subscriber sub = nh.subscribe("/scan", 10, &TurtlebotNav::lidarCallback,
                                     this);

  /**
   * @fn void lidarCallback(const sensor_msgs::LaserScan::ConstPtr&)
   * @brief A callback function to read the lidar data and
   * decide the action for turtlebot3
   *
   * @param msg
   * @return None
   */
  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // If the robot is closer to a obstacle
    if (msg->ranges[0] < 0.5) {
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 1.0;
      for (int k = 0; k < 5; k++)
        pub.publish(twist);
    }
    // If there is no obstacle
    else {
      twist.linear.x = 0.5;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      pub.publish(twist);
    }
  }

};

#endif
