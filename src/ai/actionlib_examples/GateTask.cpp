#include "GateTask.hpp"
#include <string>

GateTask::GateTask(std::string name) : _as(_nh, name, false), _action_name(name)
{
    _action_name = name;
    _as.registerGoalCallback(boost::bind(&GateTask::goalCallback, this));
    _as.registerPreemptCallback(boost::bind(&GateTask::preemptCallback, this));

    _sub = _nh.subscribe("vision/start_gate", 1, &GateTask::analysisCallback,
                         this);
    _pub = _nh.advertise<robosub::control>("control", 1);
    _as.start();
    currentState = TaskState::LOST;
    prev_yaw = 0;
}

void GateTask::goalCallback()
{
    prev_yaw = 0;
    errorGoal = _as.acceptNewGoal()->error;
}

void GateTask::preemptCallback()
{
    ROS_INFO_STREAM("" << _action_name << " preempted.");

    _as.setPreempted();
}

void GateTask::analysisCallback(const robosub::visionPosArray::ConstPtr& vision)
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

    ROS_INFO_STREAM("" << _action_name << " currently " <<
                    taskToString(currentState));

    int numFound = vision->data.size();
    int gateXPos;
    int gateYPos;
    switch (numFound)
    {
        case 0:
            msg.yaw_state = robosub::control::STATE_RELATIVE;
            msg.forward_state = robosub::control::STATE_ERROR;
            msg.forward = 0;
            msg.dive_state = robosub::control::STATE_RELATIVE;
            msg.dive = 0;

            switch(currentState)
            {
                case TaskState::LOST:
                    msg.yaw_left = 10;
                    break;
                case TaskState::SEARCHING_LEFT:
                    currentState = TaskState::SEARCHING_RIGHT;
                    msg.yaw_left = -10;
                    break;
                case TaskState::SEARCHING_RIGHT:
                    currentState = TaskState::SEARCHING_LEFT;
                    msg.yaw_left = 10;
                    break;
                case TaskState::TRACKING:
                    msg.yaw_left = 0;
                    break;
                case TaskState::COMPLETE:
                    break;
            }

            _feedback.xError = -1;
            _feedback.yError = -1;
            break;
        case 1:
            switch (currentState)
            {
                case TaskState::LOST:
                    if (vision->data[0].xPos < 0)
                    {
                        currentState = TaskState::SEARCHING_RIGHT;
                        msg.yaw_left = -10;
                    }
                    else
                    {
                        currentState = TaskState::SEARCHING_LEFT;
                        msg.yaw_left = 10;
                    }
                    break;
                case TaskState::SEARCHING_LEFT:
                case TaskState::SEARCHING_RIGHT:
                    break;
                case TaskState::TRACKING:
                    msg.yaw_state = robosub::control::STATE_RELATIVE;
                    msg.yaw_left = prev_yaw * -1;
                    if (msg.yaw_left > 0)
                    {
                        currentState = TaskState::SEARCHING_LEFT;
                    }
                    else if (msg.yaw_left < 0)
                    {
                        currentState = TaskState::SEARCHING_RIGHT;
                    }
                    else
                    {
                        if (vision->data[0].xPos < 0)
                        {
                            //Post on right disappeared, find it
                            currentState = TaskState::SEARCHING_RIGHT;
                            msg.yaw_left = -10;
                        }
                        else if (vision->data[0].xPos > 0)
                        {
                            //Post on left disappeared, find it
                            currentState = TaskState::SEARCHING_LEFT;
                            msg.yaw_left = 10;
                        }
                    }
                    break;
                case TaskState::COMPLETE:
                    break;
            }
            _feedback.xError = abs(vision->data[0].xPos);
            _feedback.yError = abs(vision->data[0].yPos);
            break;
        case 2:
            currentState = TaskState::TRACKING;

            gateXPos = (vision->data[0].xPos + vision->data[1].xPos) / 2;
            gateYPos = (vision->data[0].yPos + vision->data[1].yPos) / 2;

            if (abs(gateXPos) > errorGoal)
            {
                //Center in x direction
                msg.yaw_state = robosub::control::STATE_ERROR;
                msg.yaw_left = gateXPos * -1;
            }
            else if (abs(gateYPos) > errorGoal)
            {
                //Center in y direction
                msg.dive_state = robosub::control::STATE_ERROR;
                msg.dive = gateYPos * -1;
            }
            else
            {
                //Centered!
                if ((vision->data[0].magnitude + vision->data[0].magnitude) >
                    0.3)
                {
                    //Goal complete, we have passed through the gate
                    currentState = TaskState::COMPLETE;
                    _result.success = true;
                    ROS_INFO_STREAM("" << _action_name <<
                                    ": Passed through gate, " << "success!");
                    _as.setSucceeded(_result);
                    break;
                }
                //Go forward
                msg.forward_state = robosub::control::STATE_ERROR;
                msg.forward = 10;
            }
            _feedback.xError = abs(gateXPos);
            _feedback.yError = abs(gateYPos);

            break;
        default:
            ROS_FATAL_STREAM("Found more than 2 gate objects");
            _result.success = false;
            _as.setAborted(_result);
            break;
    }

    if (currentState == TaskState::COMPLETE)
    {
        msg.forward_state = robosub::control::STATE_ERROR;
        msg.forward = 0;
        msg.yaw_state = robosub::control::STATE_RELATIVE;
        msg.yaw_left = 0;
        msg.dive_state = robosub::control::STATE_RELATIVE;
        msg.dive = 0;
    }

    _feedback.state = taskToString(currentState);

    prev_yaw = msg.yaw_left;
    _pub.publish(msg);
    _as.publishFeedback(_feedback);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "gate_task");

    GateTask gateTask(ros::this_node::getName());

    ros::spin();

    return 0;
}
