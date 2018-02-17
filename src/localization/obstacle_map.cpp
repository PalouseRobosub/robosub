#include "localization/obstacle_map.h"
#include <map>
#include <string>

namespace robosub
{
    ObstacleMap::ObstacleMap(ros::NodeHandle &_nh) :
        nh(_nh)
    {
    }

    void ObstacleMap::InputPositionCallback(const
            robosub_msgs::ObstaclePosArray::ConstPtr &msg)
    {
        for(robosub_msgs::ObstaclePos item : msg->data)
        {
            obstacle_map[item.name] = tf::Vector3(item.x,
                    item.y, item.z);
        }
    }

    tf::Vector3 ObstacleMap::GetObstacle(std::string name)
    {
        return obstacle_map[name];
    }

    tf::Vector3 ObstacleMap::GetDistanceFromSub(tf::Vector3 sub_position,
            std::string name)
    {
        return obstacle_map[name] - sub_position;
    }

    std::string ObstacleMap::GetClosestToSub(tf::Vector3 sub_position)
    {
        double min = 100000000000.0;
        std::string name;

        for(std::map<std::string, tf::Vector3>::iterator it =
                obstacle_map.begin(); it != obstacle_map.end(); it++)
        {
            double x = it->second[0] - sub_position[0];
            double y = it->second[1] - sub_position[1];
            double z = it->second[2] - sub_position[2];

            double distance = std::sqrt( std::pow(x, 2) + std::pow(y, 2) +
                    std::pow(z, 2));

            if(distance < min)
            {
                min = distance;
                name = it->first;
            }
        }

        return name;
    }
}
