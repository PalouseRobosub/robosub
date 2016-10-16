#pragma once

#include "utility/serial.hpp"
#include <vector>


// To use the thruster controller class: 
//	1. construct local thrustercontroller object
//	2. call configure()
//	3. construct vector containing multiple objects of 
//		type thrusterVector defined in this file. 
//	4. pass said vector to setSpeed function of controller object. 
// 
// When the node terminates, the destructor should be 
// implicitly called (may want to explicitly call anyhow)
//
//
// thrusterVector object can be constructed with its constructor
// fSpeed is a normalized float value for thruster speed, -1 for full reverse
// and 1 for full forward. nThruster is the thruster number. need to figure out
// numbering convention for thrusters.
//
// TODO: figure out timing to see if 30Hz update rate is achievable, since
// the serial write is a blocking call.  
using std::vector;

namespace Thruster
{
	enum  char* cPort = "/dev/ttyUSB0";
}

typedef struct tv 
{
	tv(const float fSpeed, const unsigned int nThruster)
	{
		this->speed = fSpeed;
		this->thruster_number = nThruster;
	}
	float speed;
	unsigned int thruster_number;
}thrusterVector;

class ThrusterController
{
	public:
		ThrusterController(); // default constructor should be called
		~ThrusterController(); // hopefully will terminate thrusters upon destruction

		bool configure(); // this would try to open up the serial port to the thruster controller 
					      // return false if thruster controller not detected. 
		int SetSpeed(const vector<thrusterVector> speeds); // see below
					   //
					   //
			          // Someone need to give me some kind of 
       // data  parsed from the ROS message for an array of float(? double?) values ranging from -1 to 1, 
    // whereas -1 is full reverse, and 1 is full speed forward, also need the length of the 
        // array being passed in, or possibly define a struct/class that holds a pair of data, such as 
       // struct ThrusterSpeedVector{ float speed, int ThrusterNumber}; 
      // and pass in an array of those with the number of them. 
	private:

		rs::Serial Controller_Port;

};

void  parseNormalized(const float fSpeed, uint8_t & SendByte);
