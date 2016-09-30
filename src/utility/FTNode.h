#ifndef __ROBOSUB_FTNODE_H__
#define __ROBOSUB_FTNODE_H__

#include <ftd2xx.h>
#include <string>

namespace rs
{
	class FTNode
	{
	public:

		enum class bits_per_word : UCHAR
		{
			EIGHT = FT_BITS_8,
			SEVEN = FT_BITS_7
		};

		enum class stop_bits : UCHAR
		{
			ONE = FT_STOP_BITS_1,
			TWO = FT_STOP_BITS_2
		};

		enum class parity : UCHAR
		{
			NONE = FT_PARITY_NONE,
			ODD = FT_PARITY_ODD,
			EVEN = FT_PARITY_EVEN,
			MARK = FT_PARITY_MARK,
			SPACE = FT_PARITY_SPACE
		};

		enum class flow : USHORT
		{
			NONE = FT_FLOW_NONE,
			RTS_CTS = FT_FLOW_RTS_CTS,
			DTR_DSR = FT_FLOW_DTR_DSR,
			XON_XOFF = FT_FLOW_XON_XOFF
		};

		FTNode();
		~FTNode();
		bool open();
		bool read(unsigned int numChars, std::string& msg);
		bool write(const std::string& msg);
		void setBaudRate(DWORD baud_rate);
		void setBitsPerWord(FTNode::bits_per_word bits_per_word);
		void setStopBits(FTNode::stop_bits stop_bits);
		void setParity(FTNode::parity parity);
		void setFlowControl(FTNode::flow flow, UCHAR x_on = '\0',
							UCHAR x_off = '\0');

	private:
		FT_HANDLE m_handle;
		FT_STATUS m_status;
		DWORD m_baud_rate = 9600;
		UCHAR m_bits_per_word = static_cast<UCHAR>(bits_per_word::EIGHT);
		UCHAR m_stop_bits = static_cast<UCHAR>(stop_bits::ONE);
		UCHAR m_parity = static_cast<UCHAR>(parity::NONE);
		USHORT m_flow_control = static_cast<USHORT>(flow::NONE);
	};
}

#endif
