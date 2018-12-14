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


#include "IPC_shm.hpp"

#include "ssd_detect.h"

#define SHM_QUEUE_NAME 1

static void * cap;
static float fps = 0;
double demo_time;
static int demo_done = 0;
static double frame_time = 0;

sigset_t newset;
sigset_t zeroset;
static int sigFlag = 0;


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
//-----------------------------------------IPCshm Test-----------------------------------------
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
//    string model_file_ = "/home/onelly/model/Vel_Pes_Res18_M1_visp_deploy_bnmerge.prototxt";
//    string weights_file_ = "/home/onelly/model/1109_bias_bnmerge.caffemodel";
//    string mean_file_ = "";
//    string mean_value_ = "104,117,123";
//    Detector *detector = new Detector(model_file_, weights_file_, mean_file_, mean_value_);

//    cv::Mat img = cv::imread("/home/onelly/Pictures/test.png", -1);
//    //cv::Mat img = cv::imread(filename_picture, -1);
//    //CHECK(!img.empty()) << "Unable to decode image " << file;
//    Caffe::set_mode(Caffe::GPU);
//    std::vector<vector<float> > detections = detector->Detect(img);

//    /* Print the detection results. */
//    for (int i = 0; i < detections.size(); ++i) {
//        const vector<float>& d = detections[i];
//        // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
//        //CHECK_EQ(d.size(), 7);
//        const float score = d[2];
//        if (score >= 0.5)
//        {
//            string test;
//            string label ="class :";
//            label[5] = '0'+d[1];
//            std::cout << label<<std::endl;
//            test = std::to_string(score);
//            test = label + test;
//            cv::putText(img, test, cvPoint(static_cast<int>(d[3] * img.cols), static_cast<int>(d[4] * img.rows)), CV_FONT_HERSHEY_COMPLEX,0.8, CV_RGB(255, 0, 0),2,8,false);
//            cv::rectangle(img, cvPoint(d[3] * img.cols, d[4] * img.rows), cvPoint(d[5] * img.cols, d[6] * img.rows), cvScalar(255, 0, 0), 2, 8, 0);
//        }
//    }

//-----------------------------------------Write Test-----------------------------------------
//    printf("SHM_STRUCT size %ld\n",sizeof(SHM_STRUCT));
//    printf("IMAGEINFO size %ld\n",sizeof(IMAGEINFO));

//    key_t key;
//    IPC_shm ishm[SHM_QUEUE_NAME];

//    printf("attach shm start...%ld\n",time(0));
//    const char* name = shm_name[0];
//    key = ftok(name,0);

//    ishm[0].siOpenShm(key);
//    ishm[0].siAttShm(NULL,0);
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
//                //read
//                cv::Mat m(p_imageinfo->w,p_imageinfo->h,CV_8UC3);
//                m.data = p_imageinfo->data1;
//                m = m.reshape(0, p_imageinfo->h);

//                //process
//                cv::rectangle(m, cvPoint(0.4 * m.cols, 0.4 * m.rows), cvPoint(0.6 * m.cols, 0.6 * m.rows), cvScalar(255, 0, 0), 2, 8, 0);

//                //write
//                IplImage ipl = m;
//                p_imageinfo->h = ipl.height;
//                p_imageinfo->w = ipl.width;
//                p_imageinfo->c = ipl.nChannels;
//                p_imageinfo->s = ipl.widthStep;
//                m = m.reshape(0, 1);


//                //status change
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
//-----------------------------------------Final Test-----------------------------------------
    printf("SHM_STRUCT size %ld\n",sizeof(SHM_STRUCT));
    printf("IMAGEINFO size %ld\n",sizeof(IMAGEINFO));

    key_t key;
    IPC_shm ishm[SHM_QUEUE_NAME];

    printf("attach shm start...%ld\n",time(0));
    const char* name = shm_name[0];
    key = ftok(name,0);

    ishm[0].siOpenShm(key);
    ishm[0].siAttShm(NULL,0);
    printf("name=%s,key=%d\n",name,key);
    printf("attach shm ok...%ld\n",time(0));

    int count = 0;
    long framenum = 0;

    //detector init
    string model_file_ = "/home/onelly/model/Vel_Pes_Res18_M1_visp_deploy_bnmerge.prototxt";
    string weights_file_ = "/home/onelly/model/1109_bias_bnmerge.caffemodel";
    string mean_file_ = "";
    string mean_value_ = "104,117,123";
    Detector *detector = new Detector(model_file_, weights_file_, mean_file_, mean_value_);
    Caffe::set_mode(Caffe::GPU);

    while(!demo_done)
    {
        int i = framenum%SHM_QUEUE_NAME;
        framenum++;

        if( i == 0 )
            count++;

        if( count > SHM_NAME_NUM )
            count = 1;

        printf("detect i=%d,count=%d\n",i,count);

        SHM_STRUCT *p_addr = ishm[i].siGet_Start_ByPos(count-1);
        IMAGEINFO *p_imageinfo = (IMAGEINFO *)p_addr->shm_value;

        while(1)
        {
            if( p_imageinfo->status == 1)
            {
                //read
                cv::Mat img(p_imageinfo->w,p_imageinfo->h,CV_8UC3);
                img.data = p_imageinfo->data1;
                img = img.reshape(0, p_imageinfo->h);

                //process
                std::vector<vector<float> > detections = detector->Detect(img);
                for (int i = 0; i < detections.size(); ++i) {
                    const vector<float>& d = detections[i];
                    // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
                    //CHECK_EQ(d.size(), 7);
                    const float score = d[2];
                    if (score >= 0.5)
                    {
                        string test;
                        string label ="class :";
                        label[5] = '0'+d[1];
                        std::cout << label<<std::endl;
                        test = std::to_string(score);
                        test = label + test;
                        cv::putText(img, test, cvPoint(static_cast<int>(d[3] * img.cols), static_cast<int>(d[4] * img.rows)), CV_FONT_HERSHEY_COMPLEX,0.8, CV_RGB(255, 0, 0),2,8,false);
                        cv::rectangle(img, cvPoint(d[3] * img.cols, d[4] * img.rows), cvPoint(d[5] * img.cols, d[6] * img.rows), cvScalar(255, 0, 0), 2, 8, 0);
                    }
                }

                //write
                IplImage ipl = img;
                p_imageinfo->h = ipl.height;
                p_imageinfo->w = ipl.width;
                p_imageinfo->c = ipl.nChannels;
                p_imageinfo->s = ipl.widthStep;
                img = img.reshape(0, 1);


                //status change
                p_imageinfo->status = 3;

            }
            else
            {
                printf("Detector Waiting: status = %d, count = %d.\n",p_imageinfo->status,count);
                sleep(0);
                continue;
            }

            break;
        }
    }
    return 0;
}


