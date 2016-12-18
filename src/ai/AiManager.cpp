#include "AiManager.hpp"
#include <robosub/AIAction.h>
AiManager::AiManager()
{
    try
    {
        nh.getParam("tasks", taskList);
        
        ROS_INFO_STREAM("Type: " << taskList.getType());
    }
    catch (XmlRpc::XmlRpcException e)
    {
        ROS_ERROR_STREAM("XmlRpcException on get, code: " << e.getCode() <<
                         ": " << e.getMessage());
        exit(-1);
    }

    currentTask = 0;
    ROS_INFO_STREAM("Init done");
}

void AiManager::start()
{
    startNextTask();
}

void AiManager::completionCallback(
                        const actionlib::SimpleClientGoalState& state,
                        const robosub::AIResultConstPtr& result)
{
    std::string name = static_cast<std::string>(taskList[currentTask]["name"]);
    ROS_INFO_STREAM(name << " finished in " << state.toString() << " state");
    startNextTask();
}

void AiManager::activationCallback()
{
    ROS_INFO_STREAM("Sent goal to " <<
             static_cast<std::string>(taskList[currentTask]["name"]));
}

void AiManager::feedbackCallback(const robosub::AIFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("Feedback: " << feedback->state <<
             "X Err: " << feedback->xError <<
             "Y Err: " << feedback->yError);
}

bool AiManager::startNextTask()
{
    if (currentTask < taskList.size())
    {
        try
        {
            actionlib::SimpleActionClient<robosub::AIAction> ac(
                                                  taskList[currentTask]["node"],
                                                                true);
            ROS_INFO_STREAM("Waiting to connect to " <<
                            taskList[currentTask]["name"] << " server...");

            ac.waitForServer();

            ROS_INFO("Connected to server, sending goal.");

            robosub::AIGoal goal;
            goal.error = 10;

            ac.sendGoal(goal,
                    boost::bind(&AiManager::completionCallback, this, _1, _2),
                    boost::bind(&AiManager::activationCallback, this),
                    boost::bind(&AiManager::feedbackCallback, this, _1));

        }
        catch (XmlRpc::XmlRpcException e)
        {
            ROS_ERROR_STREAM("XmlRpcException code " << e.getCode() << ": " <<
                             e.getMessage());
            exit(-1);
        }
        currentTask++;
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_manager");

    AiManager manager;

    manager.start();
    
    ros::spin();
    //TODO: Send shutdown to control system just in case
    
    ROS_INFO_STREAM("All tasks completed.");
}
