#include "thruster_serial.hpp"

ThrusterController::ThrusterController()
{
	// I don't think anything needs to be here...
}

ThrusterController::~ThrusterController()
{
	// need to first send the off signal to the controller, then close port
	
//	this->Controller_Port.Write(...);
	this->Controller_Port.Close();
}
