#include "thruster_serial.hpp"

ThrusterController::ThrusterController()
{
	// I don't think anything needs to be here...
}

ThrusterController::~ThrusterController()
{
	// need to first send the off signal to the controller, then close port
	// data to be sent
	uint8_t closeBytes[5] = {0xaa, 0x84, 0x00, 0x70, 0x2e};

	// sequentially set output to 1500us to all channels. 
	for (int i=0; i < 12; i++)
	{
		this->Controller_Port.Write(closeBytes);
		closeBytes[2] += 1;
	}
	this->Controller_Port.Close();
}


bool ThrusterController::configure()
{
	// Open up serial port. 
	this->Controller_Port.Open("/dev/ttyUSB0", B9600);
	uint8_t ClockByte = 0xaa;
	this->Controller_Port.Write(&ClockByte, 1);

}

int ThrusterController::SetSpeed(const vector<thrusterVector> speeds)
{

	this->Controller_Port.Write(, 4);
}


// maestro controller takes speed in terms of quarter microseconds...
// but they lied on the datasheet (or doing some kind of parsing that 
// I don't understand. Output is 1500us at 0x702e, 1000us at 0x701f, 
// 2000us at 0x703e
void  parseNormalized(const float fSpeed, uint8_t & SendByte)
{	
	SendByte = 15.5*fSpeed + 46;
}
