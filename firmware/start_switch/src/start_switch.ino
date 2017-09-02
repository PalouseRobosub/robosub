/*
 * Author: Zachary Pratt
 * Version: 0.1
*/
#include <ros.h>
#include <std_msgs/Bool.h>

static constexpr int start_switch_pin = 13;

ros::NodeHandle n;

std_msgs::Bool start_switch_msg;
ros::Publisher start_switch_pub("start_switch", &start_switch_msg);

void setup()
{
    pinMode(start_switch_pin, INPUT);
    n.initNode();
    n.advertise(start_switch_pub);
}

void loop()
{
    start_switch_msg.data = (digitalRead(start_switch_pin) == LOW);

    start_switch_pub.publish(&start_switch_msg);

    n.spinOnce();
    delay(200);
}

