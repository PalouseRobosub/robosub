#include <ros/ros.h>
#include <robosub/AIAction.h>
#include "BuoyTask.hpp"

BuoyTask::BuoyTask(std::string name) : _as(_nh, name, false), _action_name(name)
{
    _action_name = name;

    _as.registerGoalCallback(boost::bind(&BuoyTask::goalCallback, this));
    _as.registerPreemptCallback(boost::bind(&BuoyTask::preemptCallback, this));

    _sub = _nh.subscribe("vision/buoys/red", 1,
                         &BuoyTask::analysisCallback, this);
    _pub = _nh.advertise<robosub::control>("control", 1);
    _as.start();
    currentState = TaskState::SEARCHING;
}

void BuoyTask::goalCallback()
{
    errorGoal = _as.acceptNewGoal()->error;
}

void BuoyTask::preemptCallback()
{
    ROS_INFO_STREAM(_action_name << " preempted.");

    _as.setPreempted();
}

void BuoyTask::analysisCallback(const robosub::visionPosArray::ConstPtr& vision)
{
    if (!_as.isActive())
    {
        return;
    }

    robosub::control msg;

    msg.roll_state = robosub::control::STATE_ABSOLUTE;
    msg.roll_right = 0;
    msg.pitch_state = robosub::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;

    ROS_INFO_STREAM(_action_name << " currently " << currentState);

    if (vision->data.empty())
    {
        if (currentState == TaskState::TRACKING)
        {
            _result.success = false;
            ROS_INFO_STREAM(_action_name << ": Lost buoy, aborting...");
            _as.setAborted(_result);
            return;
        }
        
        currentState = TaskState::SEARCHING;

        msg.yaw_state = robosub::control::STATE_RELATIVE;
        msg.yaw_left = 20;

        msg.forward_state = robosub::control::STATE_ERROR;
        msg.forward = 0;
        msg.dive_state = robosub::control::STATE_RELATIVE;
        msg.dive = 0;

        _feedback.xError = -1;
        _feedback.yError = -1;
    }
    else if (abs(vision->data[0].xPos) < errorGoal &&
             abs(vision->data[0].yPos) < errorGoal)
    {
        msg.forward_state = robosub::control::STATE_RELATIVE;
        msg.forward = 0;
        msg.yaw_state = robosub::control::STATE_RELATIVE;
        msg.yaw_left = 0;
        msg.dive_state = robosub::control::STATE_RELATIVE;
        msg.dive = 0;
        currentState = TaskState::COMPLETE;
        _result.success = true;

        ROS_INFO_STREAM(_action_name << ": Succeeded");
        _as.setSucceeded(_result);
    }
    else
    {
        currentState = TaskState::TRACKING;

        msg.yaw_state = robosub::control::STATE_ERROR;
        msg.yaw_left = vision->data[0].xPos * -1;
        msg.dive_state = robosub::control::STATE_ERROR;
        msg.dive = vision->data[0].yPos * -1;

        _feedback.xError = vision->data[0].xPos;
        _feedback.yError = vision->data[0].yPos;
        
    }

    _feedback.state = taskToString(currentState);
    
    _pub.publish(msg);
    _as.publishFeedback(_feedback);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "buoy_task");

    BuoyTask bouyTask(ros::this_node::getName());

    ros::spin();

    return 0;
}
