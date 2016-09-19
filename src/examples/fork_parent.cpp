#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <stdlib.h>
ros::Publisher pub;
int pid;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void convertCallback(const std_msgs::Float64::ConstPtr& msg)
{
	std_msgs::Float64 outmsg;

	double degrees = msg->data;
	double radians = degrees * 3.14/180;

	outmsg.data = radians;
	std::cout << "pid" << getpid() << " got message" << std::endl;

	pub.publish(outmsg);

}

void printargv(char **argv)
{
	int i = 0;
	std::cout << "environment:" << std::endl;
	while(argv[i] != 0)
	{
		std::cout << argv[i] << std::endl;
		++i;
	}	
}

int main(int argc, char **argv, char **envp)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
//	printargv(argv);
//	printargv(envp);
  ros::init(argc, argv, "fork_parent");
  ROS_INFO("parent here!");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("degrees", 1, convertCallback);
  pub = n.advertise<std_msgs::Float64>("radians", 1);

  std::cout << "forking now!" << std::endl;
  pid = fork();
  if(pid < 0) //fork failed
  {
	  std::cout << "failed to fork" << std::endl;
  }
  if(pid) //parent
  {
	  std::cout << "parent here, child pid:" << pid << std::endl;
  		ros::spin();
  }
  else //child
  {
	  std::cout << "child here, attempted to exec..." << std::endl;

	  execlp("rosrun", "rosrun", "robosub", "fork_child", (char*) NULL);

	  std::cout << "child here, exec failed" << std::endl;
  }

  return 0;
}
