#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robosub/AIAction.h>
#include <XmlRpcException.h>
#include <string>
#include <iostream>

class AiManager
{
    public:
        AiManager();
        ~AiManager() { };

        void start();
    private:
        void completionCallback(const actionlib::SimpleClientGoalState& state,
                                const robosub::AIResultConstPtr& result);
        void activationCallback();
        void feedbackCallback(const robosub::AIFeedbackConstPtr& feedback);

        bool startNextTask();

        XmlRpc::XmlRpcValue taskList;
        int currentTask;
        ros::NodeHandle nh;
};
