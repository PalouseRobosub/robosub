#ifndef __SPAWN_PROCESS_H__
#define __SPAWN_PROCESS_H__

namespace rs
{
    class SpawnProcess
    {
    public:
        SpawnProcess();
        ~SpawnProcess();
        void Spawn(std::string cmd, std::string args="");

    private:
        pid_t m_pid;
        bool spawned;
    };
};

#endif
