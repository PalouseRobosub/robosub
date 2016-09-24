#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
ros::Publisher pub;


int main(int argc, char **argv)
{
 ros::init(argc, argv, "talker");

 ros::NodeHandle n;

  pub = n.advertise<std_msgs::String>("chatter", 1);

  
  int loop_frequency;
  if(!n.getParam("loop_rate", loop_frequency))
  {
	  ROS_INFO("frequency parameter not found, using default");
	  loop_frequency = 1;
  }
  ROS_INFO("frequency: %d", loop_frequency);
  ros::Rate rate(loop_frequency);
  while(ros::ok())
  {
	  std_msgs::String msg;
	  std::string mstr;
	  n.getParamCached("message", mstr);
	  msg.data = mstr;
	  pub.publish(msg);

	  ros::spinOnce();
	  rate.sleep();
  }

  return 0;
}
