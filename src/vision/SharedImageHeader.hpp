#ifndef SHAREDIMAGEHEADER_H
#define SHAREDIMAGEHEADER_H

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>

typedef struct
{
    int fd;
    sem_t *sem;
    cv::Size size;
    int type;
    int data_size;
    void *data;
} SharedImageHeader;

#endif
