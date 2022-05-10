#ifndef WALKER_H
#define WALKER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Walker {
public:
  //Tunable parameters
  static constexpr double FORWARD_SPEED = 0.3;
  static constexpr float DIST_FROM_OBSTACLE = 0.3f; // Should be smaller
          // than sensor_msgs::LaserScan::range_max
  Walker();
  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub; // Publisher to the robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
  bool keepMoving; // Indicates whether the robot should continue moving
  bool isObstacleInFront;

  void frontScan(const sensor_msgs::LaserScan::ConstPtr& scan);
  void findFreeWay(const sensor_msgs::LaserScan::ConstPtr& scan);
  void checkingForObstacles(const sensor_msgs::LaserScan::ConstPtr& scan);
  void moveForward();
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // WALKER_H
