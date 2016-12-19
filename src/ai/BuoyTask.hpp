#ifndef BUOYTASK_HPP
#define BUOYTASK_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robosub/AIAction.h>
#include <robosub/visionPosArray.h>
#include <string>
#include <robosub/control.h>


class BuoyTask
{
    public:
        BuoyTask(std::string name);
        ~BuoyTask(void) { }

        void goalCallback();
        void preemptCallback();
        void analysisCallback(const robosub::visionPosArray::ConstPtr& msg);

    protected:
        enum TaskState
        {
            SEARCHING,
            TRACKING,
            COMPLETE
        };

        ros::NodeHandle _nh;
        actionlib::SimpleActionServer<robosub::AIAction> _as;
        std::string _action_name;
        robosub::AIFeedback _feedback;
        robosub::AIResult _result;
        ros::Subscriber _sub;
        ros::Publisher _pub;

        TaskState currentState;

        double errorGoal;

        std::string taskToString(TaskState &ts)
        {
            switch (ts)
            {
                case TaskState::SEARCHING:
                    return "SEARCHING";
                case TaskState::TRACKING:
                    return "TRACKING";
                case TaskState::COMPLETE:
                    return "COMPLETE";
                default:
                    return "UNKNOWN";
            }
        }
};
#endif //BUOYTASK_HPP
