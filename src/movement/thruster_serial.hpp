#include "utility/serial.hpp"
#include <vector>

typedef struct 
{
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

}

void  parseNormalized(const float fSpeed, uint8_t & SendByte);
