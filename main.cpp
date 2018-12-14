#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
//caffe include
#include <caffe/caffe.hpp>
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <utility>
#include <vector>

#include "IPC_shm.hpp"

using namespace caffe;
//#define CPU_ONLY

#define SHM_QUEUE_NAME 1

static void * cap;
static float fps = 0;
double demo_time;
static int demo_done = 0;
static double frame_time = 0;

sigset_t newset;
sigset_t zeroset;
static int sigFlag = 0;

//caffe global
shared_ptr<Net<float> > net_;

const char shm_name[SHM_QUEUE_NAME][32] =
{
    "/dev/shm/imageshm001"/*,
    "/dev/shm/imageshm002",
    "/dev/shm/imageshm003",
    "/dev/shm/imageshm004",
    "/dev/shm/imageshm005",
    "/dev/shm/imageshm006",
    "/dev/shm/imageshm007",
    "/dev/shm/imageshm008"*/
};

// The conversion here is very slow
/*typedef struct
{
    char year[8];
    char month[4];
    char day[4];
    unsigned int areaid;
    unsigned int batch;
    unsigned int index;
    unsigned int number;
    unsigned int sn;
    volatile   int status;
    unsigned int datalen1;
    unsigned int datalen2;
    unsigned long frames;
    unsigned long frametime;
    char pos[128];
    unsigned int h;
    unsigned int w;
    unsigned int c;
    unsigned int s;
    float data1[(SHM_VALUE_SIZE-208)/(2*4)];
    float data2[(SHM_VALUE_SIZE-208)/(2*4)];
}IMAGEINFO;*/

typedef struct
{
    char year[8];
    char month[4];
    char day[4];
    unsigned int areaid;
    unsigned int batch;
    unsigned int index;
    unsigned int number;
    unsigned int sn;
    volatile   int status;
    unsigned int datalen1;
    unsigned int datalen2;
    unsigned long frames;
    unsigned long frametime;
    char pos[128];
    unsigned int h;
    unsigned int w;
    unsigned int c;
    unsigned int s;
    unsigned char data1[(SHM_VALUE_SIZE-208)/2];
    unsigned char data2[(SHM_VALUE_SIZE-208)/2];
}IMAGEINFO;

typedef struct {
    int w;
    int h;
    int c;
    float *data;
} image;



double what_time_is_it_now()
{
    struct timeval time;

    if (gettimeofday(&time,NULL))
    {
        return 0;
    }

    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}


// g++ simplest_rtmp_receive.cpp -o simplest_rtmp_receive IPC_shm.o -std=c++0x -Wall -fpermissive -Wcpp -I /usr/include/jsoncpp `pkg-config --libs --cflags opencv` -lcurl -L /usr/local/lib -ljsoncpp
int main(int argc, char* argv[])
{

//    printf("SHM_STRUCT size %ld\n",sizeof(SHM_STRUCT));
//    printf("IMAGEINFO size %ld\n",sizeof(IMAGEINFO));


//    key_t key;
//    IPC_shm ishm[SHM_QUEUE_NAME];

//    printf("attach shm start...%ld\n",time(0));
//        const char* name = shm_name[0];
//        key = ftok(name,0);

//        ishm[0].siOpenShm(key);
//        ishm[0].siAttShm(NULL,0);
//    printf("name=%s,key=%d\n",name,key);

//    printf("attach shm ok...%ld\n",time(0));




//    int count = 0;
//    long framenum = 0;

//    while(!demo_done)
//    {
//        int i = framenum%SHM_QUEUE_NAME;
//        framenum++;

//        if( i == 0 )
//            count++;

//        if( count > SHM_NAME_NUM )
//            count = 1;

//        printf("detect i=%d,count=%d\n",i,count);

//        SHM_STRUCT *p_addr = ishm[i].siGet_Start_ByPos(count-1);
//        IMAGEINFO *p_imageinfo = (IMAGEINFO *)p_addr->shm_value;

//        while(1)
//        {
//            if( p_imageinfo->status == 1)
//            {
//                p_imageinfo->status = 3;
//            }
//            else
//            {
//                printf("Detector Waiting: status = %d, count = %d.\n",p_imageinfo->status,count);
//                sleep(0);
//                continue;
//            }

//            break;
//        }
//    }

//-----------------------------------------Detect Test-----------------------------------------
    #ifdef CPU_ONLY
        Caffe::set_mode(Caffe::CPU);
    #else
        Caffe::set_mode(Caffe::GPU);
        Caffe::SetDevice(0);
    #endif

    return 0;
}


