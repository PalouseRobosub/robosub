#include "movemet/maestro_class.hpp"

ThrusterController::~ThrusterController()
{
	// need to first send the off signal to the controller, then close port
	// data to be sent
	uint8_t closeBytes[5] = {0xaa, 0x84, 0x00, 0x70, 0x2e};

	// sequentially set output to 1500us to all channels. 
	for (int i=0; i < 12; i++)
	{
		this->Controller_Port.Write(closeBytes, 5);
		closeBytes[2] += 1;
	}
	this->Controller_Port.Close();
}


bool ThrusterController::configure()
{
	// Open up serial port. 
	this->Controller_Port.Open(tc::cPort, B9600);
	uint8_t ClockByte = 0xaa;
	this->Controller_Port.Write(&ClockByte, 1);
	// is there a way to know if port successfully opened?
	return true;
}

// takes a vector of thrusterVectors and sends signals to specific thrusters.
// see definition of thrusterVector struct definition
int ThrusterController::SetSpeed(const vector<thrusterVector> speeds)
{
	uint8_t buf[4] = {0x84, 0x00, 0x70, 0x2e};
	
	// if 60 signals were received
	if (signalCount >= 60)
	{
		// send reset signal to all esc's 
		for (int i=0; i<8; i++)
		{
			this->Controller_Port.Write(buf,4);
			buf[1]++;
		}
		this->signalCount = 0;
	}
	for (int i=0; i < speeds.size(); i++)
	{
		buf[1] = speeds[i].thruster_number;
		parseNormalized(speeds[i].speed, &(buf[2]));
		this->Controller_Port.Write(buf, 4);
		this->signalCount ++;
	}
}


// SendByte[1] is most significant byte, [0] is least significant byte
// unit is in 1/8 us
// TODO: Figure out a range of speeds such that the thruster is not 
// running at a million rpm. 
void  ThrusterController::parseNormalized(const float fSpeed, uint8_t* SendByte)
{	
	// gets the speed in terms of 1/8 us
	uint16_t speed = (1500 + fSpeed * 500) * 8;
	// to change maximum speed, change the 500 value in here. 
	*SendByte = 0xFF & speed;
	*(SendByte+1) = speed >> 8;	
}
