#include <cmath>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"
#include "tf/transform_datatypes.h"

/*
/rs_accel_data   855 msgs    : geometry_msgs/Vector3
/rs_bno_data     848 msgs    : geometry_msgs/Quaternion
/rs_depth_data   852 msgs    : std_msgs/Float32
/rs_mag_data     845 msgs    : geometry_msgs/Vector3
*/

class LocalizationSystem
{
public:
    LocalizationSystem(double _dt);

    void InputOrientation(geometry_msgs::Quaternion msg);
    void InputAccel(geometry_msgs::Vector3 msg);
    void InputDepth(std_msgs::Float32 msg);

    void SetPosition(double _x, double _y, double _z);
    void SetVelocity(double _x, double _y, double _z);

    // If needed
    //void InputOrientation(geometry_msgs::Quaternion::ConstPtr msg);
    //void InputAccel(geometry_msgs::Vector3::ConstPtr msg);
    //void InputDepth(std_msgs::Float32::ConstPtr msg);

    void Update();

private:
    void FindLinearAccel();

    double dt;

    bool new_orientation;
    bool new_accel;
    bool new_depth;

    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3 accel;
    std_msgs::Float32 depth;

    double pos_x;
    double pos_y;
    double pos_z;

    double vel_x;
    double vel_y;
    double vel_z;

    double lin_accel_x;
    double lin_accel_y;
    double lin_accel_z;

};
