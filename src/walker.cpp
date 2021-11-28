#include "turtlebot_vacuum/walker.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "roomba_nav");

  TurtlebotNav turtlebotNav;

  ros::spin();

  return 0;

}

