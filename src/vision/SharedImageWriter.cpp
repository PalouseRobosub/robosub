#include "SharedImageWriter.hpp"

namespace rs
{
    SharedImageWriter::SharedImageWriter(string name, Mat image)
    {
        unsigned long data_size = image.total() * image.elemSize();

        // create shared image header
        header_name = "robosub_" + name;
        int fd = shm_open(header_name.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IRWXU);
        if(fd <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to create shm for object: " + header_name);
            ros::shutdown();
        }
        header = new SharedImageHeader();
        header->fd = fd;
        header->size = image.size();
        header->type = image.type();
        header->data_size = data_size;

        // create semaphore
        sem_t *sem = sem_open(header_name.c_str(), O_CREAT, S_IRWXU, 1);
        if(sem <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to create sem for object: " + header_name);
            ros::shutdown();
        }
        header->sem = sem;

        // allocate memory for image
        int total_size = sizeof(SharedImageHeader) + data_size;
        ftruncate(fd, total_size);

        // put first frame into memory
        void *mem = mmap(0, total_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0); // let os create buffer
        if(mem <= 0)
        {
            ROS_FATAL_STREAM(strerror(errno));
            ROS_FATAL_STREAM("Failed to create memory for object: " + name);
            ros::shutdown();
        }
        header->data = (char*)mem + sizeof(SharedImageHeader); // set sim data segment to data segment of mem
        sem_wait(header->sem); // lock memory
        memcpy(mem, header, sizeof(SharedImageHeader)); // copy header
        memcpy(header->data, image.data, data_size);    // copy data
        sem_post(header->sem); // unlock memory
    }

    SharedImageWriter::~SharedImageWriter()
    {
        // delete sem
        sem_close(header->sem);

        // delete shm
        munmap(header->data, sizeof(SharedImageHeader) + header->data_size);
        close(header->fd);
    }

    void SharedImageWriter::Write(Mat image)
    {
        sem_wait(header->sem); // lock memory
        memcpy(header->data, image.data, header->data_size); // write
        sem_post(header->sem); // unlock memory
    }

    void SharedImageWriter::operator<<(Mat image)
    {
        sem_wait(header->sem); // lock memory
        memcpy(header->data, image.data, header->data_size); // write
        sem_post(header->sem); // unlock memory
    }
};
