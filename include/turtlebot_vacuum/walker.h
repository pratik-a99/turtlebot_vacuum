#ifndef ROOMBA_INCLUDE_TURTLEBOT_VACUUM_WALKER_H_
#define ROOMBA_INCLUDE_TURTLEBOT_VACUUM_WALKER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class TurtlebotNav {
 private:
  geometry_msgs::Twist twist;

  ros::NodeHandle nh;

 public:

  ros::Publisher pub = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 10);

  ros::Subscriber sub = nh.subscribe("/scan", 10, &TurtlebotNav::lidarCallback,
                                     this);

  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    if (msg->ranges[0] < 0.5) {
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 1.0;
      for (int k = 0; k < 5; k++)
        pub.publish(twist);
    } else {
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
