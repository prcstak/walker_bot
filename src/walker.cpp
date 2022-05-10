#include "Walker.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
using namespace std;
Walker::Walker()
{
    keepMoving = true;
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);   
    laserSub = node.subscribe("scan", 1, &Walker::callback, this);
}

void Walker::moveForward() {
    geometry_msgs::Twist vel;
    vel.angular.z = 0.0;
    vel.linear.x = (-1)*FORWARD_SPEED;
    commandPub.publish(vel);
}

void Walker::findFreeWay(const sensor_msgs::LaserScan::ConstPtr& scan){
    int minLeft = 240;
    int maxLeft = 300;
    int minRight = 60;
    int maxRight = 120;
    int leftSum = 0;
    int rightSum = 0;
    int rearSum = 0;
    int leftInfSum = 0;
    int rightInfSum = 0;
    int rearInfSum = 0;
    int k = 0;
    float inf = std::numeric_limits<float>::infinity();
    while (minRight + k < maxRight)
    {
        k+=1;
        if (scan->ranges[minRight + k] == inf){
            rightInfSum += 1;
        }
        if (scan->ranges[maxLeft - k] == inf){
            leftInfSum += 1;
        }

        if(k < 30){
            if (scan->ranges[30 - k] == inf){
                rearInfSum += 1;
            }
            if (scan->ranges[330 + k] == inf){
                rearInfSum += 1;
            }
        }

        if (scan->ranges[minRight + k] != inf){
            rightSum += scan->ranges[minRight + k];
        }
        if (scan->ranges[maxLeft - k] != inf){
            leftSum += scan->ranges[maxLeft - k];
        }

        if(k < 30){
            if (scan->ranges[30 - k] != inf){
                rearSum += scan->ranges[30 - k];
            }
            if (scan->ranges[330 + k] != inf){
                rearSum += scan->ranges[330 + k];
            }
        }
    }
    
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    if(rightInfSum != leftInfSum){
        if(rearInfSum > rightInfSum && rearInfSum > leftInfSum){
            vel.linear.x = 1*FORWARD_SPEED;
        }
        if(rightInfSum < leftInfSum){
            vel.angular.z = 1.0;
        }
        if(rightInfSum > leftInfSum){
            vel.angular.z = -1.0;
        }
    }
    else{
        if(rightSum != leftSum){
             if(rightSum < leftSum){
                vel.angular.z = 1.0;
            }
            if(rightSum > leftSum){
                vel.angular.z = -1.0;
            }
        }
        else{
             vel.linear.x = 1*FORWARD_SPEED;
        }
       
    }
    
    commandPub.publish(vel);
    keepMoving = true;
}

void Walker::checkingForObstacles(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(isObstacleInFront){
        findFreeWay(scan);
    }
    else{
        moveForward();
    }
}

void Walker::frontScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int minIndex = 135;
    int maxIndex = 225;
	int	midIndex = 180;
    int k = 1;
    isObstacleInFront = false;

    while (minIndex + k != midIndex){
        if(scan->ranges[minIndex + k] < DIST_FROM_OBSTACLE){
            isObstacleInFront = true;
            break;
        }
        else if(scan->ranges[maxIndex - k] < DIST_FROM_OBSTACLE){
            isObstacleInFront = true;
            break;
        }
        k+=1;
    }
    Walker::checkingForObstacles(scan);  
}

void Walker::callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    frontScan(scan);
}

void Walker::startMoving()
{
    ros::Rate rate(10);
    while (ros::ok() && keepMoving) {
        ros::spinOnce();
        rate.sleep();
    }
}
