#ifndef OBSTACLE_MAP_H
#define OBSTACLE_MAP_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <map>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "robosub_msgs/ObstaclePos.h"
#include "robosub_msgs/ObstaclePosArray.h"

namespace robosub
{
class ObstacleMap
{
public:
    ObstacleMap(ros::NodeHandle &_nh);
    tf::Vector3 GetObstacle(std::string name);
    tf::Vector3 GetDistanceFromSub(tf::Vector3 sub_position, std::string name);
    std::string GetClosestToSub(tf::Vector3 sub_position);

private:
    ros::Subscriber obstacle_sub;
    std::map<std::string, tf::Vector3> obstacle_map;
    ros::NodeHandle &nh;

    void InputPositionCallback(const robosub_msgs::ObstaclePosArray::ConstPtr
                                    &msg);
};
}

#endif // OBSTACLE_MAP_H
