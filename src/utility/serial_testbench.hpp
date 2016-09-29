#include "spawn_process.hpp"

namespace rs
{
    class SerialTB
    {
    public:
        SerialTB();
        ~SerialTB();

        //Start() takes in the port the UUT uses, and returns
        //the name of the testing port to use
        std::string Start(std::string port);

    private:
        SpawnProcess socat_proc;
        bool started;
    };

};
