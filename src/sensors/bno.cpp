#include "sensors/Bno055.h"
#include "utility/serial.hpp"
#include <iostream>
#include <cstdint>
#include "ros/ros.h"

using namespace std;

using namespace rs;

int main()
{
    ros::Time::init();

    Serial port;
    port.Open("/dev/ttyUSB0", 115200);

    Bno055 bno(port);

    int ret = bno.init();
    if (ret)
    {
        cout << "Bno055::init() returned " << ret << endl;
    }

    ret = bno.setOperationMode(Bno055::OperationMode::Ndof);
    if (ret)
    {
        cout << "Bno055::setOperationMode() returned " << ret << endl;
    }

    int16_t w, x, y, z;
    while (1)
    {
        sleep(1);
        ret = bno.readQuaternion(w,x,y,z);
        if (ret)
        {
            cout << "Bno055::readQuaternion() returned " << ret << endl;
        }
    }
}

