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

#include "robosub/ObstaclePos.h"
#include "robosub/ObstaclePosArray.h"

namespace robosub
{
    class ObstacleMap
    {
    public:
        ObstacleMap(ros::NodeHandle *nh);
        tf::Vector3 GetObstacle(std::string name);
        tf::Vector3 GetDistanceFromSub(tf::Vector3 sub_position, std::string name);
        std::string GetClosestToSub(tf::Vector3 sub_position);

    private:
        ros::Subscriber obstacle_sub;
        std::map<std::string, tf::Vector3> obstacle_map;

        void obstacle_position_callback(const robosub::ObstaclePosArray::ConstPtr
                                        &msg);
    };
}

#endif
