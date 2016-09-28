#include "gtest/gtest.h"
#include "movement/ThrusterBlueESC.hpp"
#include "utility/Debug.hpp"
#include "communication/Communicator.hpp"
#include "communication/packets/thruster_packet.hpp"
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <string>
#include "serial_testbench.hpp"


using namespace rs;
using std::cerr;
using std::endl;

class SpawnProcess
{
	public:
		SpawnProcess(string cmd, string args="")
		{
			m_pid = fork();
			if(m_pid < 0) //fork failed
			{
				cerr << "failed to fork, error: " << strerror(errno) << endl;
				exit(1);
			}
			else if (m_pid  == 0) //child
			{
				char *arg_ptrs[64];

				arg_ptrs[0] = const_cast<char*>(cmd.c_str());
				int i=1;
				if(args != "")
				{
					arg_ptrs[i] = strtok(const_cast<char*>(args.c_str()), " ");
					while(arg_ptrs[++i] = strtok(NULL, " "));
				}
				arg_ptrs[i] = 0;

				//cout << "args:" << endl;
				//for(i=0; arg_ptrs[i] != NULL; ++i)
				//{
				//	cout << " " << arg_ptrs[i] << endl;
				//}
				
				execvp(cmd.c_str(), arg_ptrs);
				cerr << "failed to exec \"" << cmd << "\", error: " << strerror(errno) << endl;
				exit(1);
			}
			else //parent
			{
				//nothing to do
			}
		}

		~SpawnProcess()
		{
				kill(m_pid, SIGTERM);
		}


	private:
		pid_t m_pid;
};


TEST(BlueESCTest, test1)
{
	SpawnProcess socat_proc("/usr/bin/socat", "pty,raw,echo=0,link=UUT_pts pty,raw,echo=0,link=test_pts");
	SpawnProcess broker_proc("bin/broker");
	usleep(50000);//wait for processes to boot up
	SpawnProcess thruster_proc("bin/thruster_blue_esc");
	usleep(50000);//wait for processes to boot up

	Communicator com("thrusterTestDriver");

	vector<double> thruster_commands;

	thruster_commands.push_back(3);
	thruster_commands.push_back(2);
	thruster_commands.push_back(1);
	thruster_packet tp(thruster_commands);
	com.send_message(message("control", "thruster", "thruster", tp.whole).whole);
	PacketzierTestbench ptbrx("UUT_pts");

	uint8_t* expected_serial_data = (uint8_t*)"apple";
	ASSERT_EQ( 0, ptbrx.check_receive_packet(test_serial_data, sizeof(test_serial_data)));


}

