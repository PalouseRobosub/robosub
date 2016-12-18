#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robosub/AIAction.h>
#include <robosub/visionPosArray.h>
#include <string>
#include <robosub/control.h>


class GateTask
{
    public:
        GateTask(std::string name);
        ~GateTask(void) { };

        void goalCallback();
        void preemptCallback();
        void analysisCallback(const robosub::visionPosArray::ConstPtr& msg);

    protected:
        
        enum class TaskState
        {
            LOST,
            SEARCHING_LEFT,
            SEARCHING_RIGHT,
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
        int prev_yaw;

        double errorGoal;

        std::string taskToString(TaskState &ts)
        {
            switch (ts)
            {
                case TaskState::LOST:
                    return "LOST";
                case TaskState::SEARCHING_LEFT:
                    return "SEARCHING_LEFT";
                case TaskState::SEARCHING_RIGHT:
                    return "SEARCHING_RIGHT";
                case TaskState::TRACKING:
                    return "TRACKING";
                case TaskState::COMPLETE:
                    return "COMPLETE";
                default:
                    return "UNKNOWN";
            }
        }
};

