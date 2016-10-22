#pragma once

#include "utility/serial.hpp"
#include <vector>
#include <thread>
#include <chrono>

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
// TODO: figure out timing to see if 30Hz update rate is achievable
using std::vector;

namespace Thruster
{
	enum uint8_t resetSignals[2] = {0x70, 0x2e};

	// temporary - will switch to udev rule for FT232 chip when interface board is 
	// complete. 
	enum  char* cPort = "/dev/ttyUSB0";
}

// custom vector that stores two values: the thruster number and normalized 
// speed of corresponding thruster [-1,1]
typedef struct tv 
{
	// constructor
	tv(const float fSpeed, const unsigned int nThruster)
	{
		this->speed = fSpeed;
		this->thruster_number = nThruster;
	}
	float speed;
	unsigned int thruster_number;
}thrusterVector;


// ===================================================================================
// main thruster controller class
// ===================================================================================
class ThrusterController
{
	public:
		ThrusterController(); // default constructor should be called
		~ThrusterController(); // hopefully will terminate thrusters upon destruction

		bool configure(); // this would try to open up the serial port to the thruster controller 
					      // return false if thruster controller not detected. Currently 
						  // only returning true
		int SetSpeed(const vector<thrusterVector> speeds); // pass in a vector of the thrusterVector
														   // defined above, use these values to assign
														   // thruster speeds. 

	private:

		rs::Serial Controller_Port;

		// counter to send reset signal. Assuming 30Hz update rate, every once in 60
		// signals will be discarded to send a reset signal to all ESC's. For details, 
		// see palouserobosub.eecs.wsu.edu/wiki/ee/thrusters/start#known_issues
		uint8_t sigalCount;

		// helper function to convert from normalized speed to a byte value sent to maestro. 
		void  parseNormalized(const float fSpeed, uint8_t* SendByte);
};

