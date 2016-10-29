#include "SharedImageReader.hpp"

namespace rs
{
    SharedImageReader::SharedImageReader(string name)
    {
        header_name = "/robosub_" + name;
        header = new SharedImageHeader();
        int fd = shm_open(header_name.c_str(), O_RDONLY, S_IRUSR);
        if(fd <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to open shm for object: " + header_name);
            ros::shutdown();
        }

        // pull shared image header
        void *mem = mmap(0, sizeof(SharedImageHeader), PROT_READ, MAP_SHARED, fd, 0); // let os create buffer
        if(mem <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to map mem for object: " + header_name);
            ros::shutdown();
        }
        memcpy(header, mem, sizeof(SharedImageHeader));

        // resize for data
        if(munmap(mem, sizeof(SharedImageHeader)) < 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to unmap mem for object: " + header_name);
            ros::shutdown();
        }
        mem = mmap(0, sizeof(SharedImageHeader) + header->data_size, PROT_READ, MAP_SHARED, fd, 0); // let os create buffer
        if(mem <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to map mem for object: " + header_name);
            ros::shutdown();
        }
        header->fd = fd;
        header->data = (char*)mem + sizeof(SharedImageHeader);

        // map sem
        sem_t *sem = sem_open(header_name.c_str(), O_CREAT, S_IRWXU, 1);
        if(sem <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to open sem for object: " + header_name);
            ros::shutdown();
        }
        header->sem = sem;
    }

    SharedImageReader::~SharedImageReader()
    {
        // delete sem
        sem_close(header->sem);

        // delete shm
        munmap(header->data, sizeof(SharedImageHeader) + header->data_size);
        close(header->fd);
    }

    Mat SharedImageReader::Read()
    {
        char *data = new char[header->data_size]; // allocate new image buffer
        sem_wait(header->sem); // lock memory
        memcpy(data, header->data, header->data_size); // write
        sem_post(header->sem); // unlock memory
        Mat m(header->size, header->type);
		memcpy(m.data, data, header->data_size);
		delete data;
        return m;
    }

    void SharedImageReader::operator>>(Mat image)
    {
        char *data = new char[header->data_size]; // allocate new image buffer
        sem_wait(header->sem); // lock memory
        memcpy(data, header->data, header->data_size); // write
        sem_post(header->sem); // unlock memory
        const Mat m(header->size, header->type, data);
        image = m.clone();
    }
};
