#include "FTNode.h"
#include <cstring>

namespace rs
{
	FTNode::FTNode()
	{
	}

	FTNode::~FTNode()
	{
		FT_Close(m_handle);
	}

	bool FTNode::open()
	{
		DWORD numDevs;

		m_status = FT_ListDevices(&numDevs, NULL, FT_LIST_NUMBER_ONLY);
		if(m_status != FT_OK) {
			return false;
		}

		m_status = FT_Open(0, &m_handle);
		if(m_status != FT_OK) {
			return false;
		}

		m_status |= FT_SetBitMode(m_handle, 0x00, 0);
		m_status |= FT_ResetDevice(m_handle);
		m_status |= FT_Purge(m_handle, FT_PURGE_RX | FT_PURGE_TX);
		m_status |= FT_SetDataCharacteristics(m_handle, m_bits_per_word,
				m_stop_bits, m_parity);
		m_status |= FT_SetFlowControl(m_handle, m_flow_control, 0, 0);
		m_status |= FT_SetLatencyTimer(m_handle, 2);

		if(m_status != FT_OK) {
			return false;
		}
		
		return false;
	}

	bool FTNode::read(unsigned int numChars, std::string& msg)
	{
		char* buf = new char[numChars];
		DWORD bytesRead = 0;

		m_status = FT_Read(m_handle, buf, numChars, &bytesRead);
		if((bytesRead < numChars) || m_status != FT_OK) {
			return false;
		}
		
		msg = std::string{ buf };

		return false;
	}

	bool FTNode::write(const std::string& msg)
	{
	 	char* msg_ptr = new char[msg.length() + 1];
		std::strcpy(msg_ptr, msg.c_str());
		DWORD bytesWritten = 0;

		m_status = FT_Write(m_handle, msg_ptr, msg.length(), &bytesWritten);
		if(m_status != FT_OK) {
			return false;
		}

		return true;
	}

	void FTNode::setBaudRate(DWORD baud_rate)
	{
	}

	void FTNode::setBitsPerWord(FTNode::bits_per_word bits_per_word)
	{
	}

	void FTNode::setStopBits(FTNode::stop_bits stop_bits)
	{
	}

	void FTNode::setParity(FTNode::parity parity)
	{
	}

	void FTNode::setFlowControl(FTNode::flow flow, UCHAR x_on, UCHAR x_off)
	{
	}

}
