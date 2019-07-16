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

//yolov3
#include<yolov3_detect.h>

//IPC_shm
#include "IPC_shm.hpp"

//ssd
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
    "/data/vdetection/0/imageshm0"/*,
    "/data/vdetection/0/imageshm1",
    "/data/vdetection/0/imageshm2",
    "/data/vdetection/0/imageshm3",
    "/data/vdetection/0/imageshm4",
    "/data/vdetection/0/imageshm5",
    "/data/vdetection/0/imageshm6",
    "/data/vdetection/0/imageshm7"*/
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
    unsigned int frames;
    unsigned int poslen;
    unsigned long frametime;
    unsigned int fps;
    unsigned int detlen;
    char pos[128];
    char recognition[1024000];
    unsigned int h;
    unsigned int w;
    unsigned int c;
    unsigned int s;
    unsigned char data1[(SHM_VALUE_SIZE - 216 - 1024000)/2];
    unsigned char data2[(SHM_VALUE_SIZE - 216 - 1024000)/2];
}IMAGEINFO;

//typedef struct {
//    int w;
//    int h;
//    int c;
//    float *data;
//} image;

DEFINE_string(szModelFile, "./network.prototxt",
    "For creating the network.");
DEFINE_string(szWeights, "./network.caffemodel",
    "To fill the network with.");
DEFINE_string(szBasePath, "data",
    "Base path of shared memory keys.");
DEFINE_string(szCameraID, "0",
    "Camera ID to be used.");
DEFINE_string(szTaskID, "0",
    "Task index of recent program.");
DEFINE_int32(iGPUID, 0,
    "GPU to be used.");
DEFINE_double(dConfThresh, 0.4,
    "Only store detections with score higher than the threshold.");

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
//-----------------------------------------GFLAGS Test-----------------------------------------
    ::google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    string model_file = FLAGS_szModelFile;
    string weights_file = FLAGS_szWeights;
    string base_path = FLAGS_szBasePath;
    string camera_id = FLAGS_szCameraID;
    string task_id = FLAGS_szTaskID;
    int gpu_id = FLAGS_iGPUID;
    double thresh = FLAGS_dConfThresh;

    string key_path = "/"+base_path+"/vdetection/"+camera_id+"/imageshm"+task_id;

    printf("key_path:%s\n",key_path.c_str());
    printf("model_file:%s\nweights_file:%s.\n\n",model_file.c_str(),weights_file.c_str());
    printf("using GPU %d\n",gpu_id);
    printf("threshold:%.1f%%\n\n",thresh*100);

//    printf("mean_file: %s,mean_value: %s. \n",mean_file.c_str(),mean_value.c_str());
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
//-----------------------------------------First Test-----------------------------------------
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

//    //detector init
//    string model_file_ = "/home/onelly/model/Vel_Pes_Res18_M1_visp_deploy_bnmerge.prototxt";
//    string weights_file_ = "/home/onelly/model/1109_bias_bnmerge.caffemodel";
//    string mean_file_ = "";
//    string mean_value_ = "104,117,123";
//    Detector *detector = new Detector(model_file_, weights_file_, mean_file_, mean_value_);
//    Caffe::set_mode(Caffe::GPU);

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
//                cv::Mat img(p_imageinfo->w,p_imageinfo->h,CV_8UC3);
//                img.data = p_imageinfo->data1;
//                img = img.reshape(0, p_imageinfo->h);

//                //process
//                std::vector<vector<float> > detections = detector->Detect(img);
//                for (int i = 0; i < detections.size(); ++i) {
//                    const vector<float>& d = detections[i];
//                    // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
//                    //CHECK_EQ(d.size(), 7);
//                    const float score = d[2];
//                    if (score >= 0.5)
//                    {
//                        string test;
//                        string label ="class :";
//                        label[5] = '0'+d[1];
//                        std::cout << label<<std::endl;
//                        test = std::to_string(score);
//                        test = label + test;
//                        cv::putText(img, test, cvPoint(static_cast<int>(d[3] * img.cols), static_cast<int>(d[4] * img.rows)), CV_FONT_HERSHEY_COMPLEX,0.8, CV_RGB(255, 0, 0),2,8,false);
//                        cv::rectangle(img, cvPoint(d[3] * img.cols, d[4] * img.rows), cvPoint(d[5] * img.cols, d[6] * img.rows), cvScalar(255, 0, 0), 2, 8, 0);
//                    }
//                }

//                //write
//                IplImage ipl = img;
//                p_imageinfo->h = ipl.height;
//                p_imageinfo->w = ipl.width;
//                p_imageinfo->c = ipl.nChannels;
//                p_imageinfo->s = ipl.widthStep;
//                img = img.reshape(0, 1);


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
//-----------------------------------------Detect Test(YOLOv3)-------------------------------
//    string model_file_ = "/home/onelly/tsq/caffe-yolov3-master/yolov3.prototxt";
//    string weights_file_ = "/home/onelly/tsq/caffe-yolov3-master/yolov3.caffemodel";
//    yolov3_detect *detector = new yolov3_detect(model_file_,weights_file_);

//    //onelly read image
//    cv::Mat img = cv::imread("/home/onelly/Pictures/test.jpg", -1);
//    //cv::Mat img = cv::imread(filename_picture, -1);
//    CHECK(!img.empty()) << "Unable to decode image " << "/home/onelly/Pictures/test.jpg";

//    //onelly detect
//    Caffe::set_mode(Caffe::GPU);
//    IplImage ipl = img;
//    std::vector<vector<int> > detections = detector->Detect(img,ipl.width,ipl.height,ipl.nChannels,ipl.widthStep);
//    //ontlly class left right top bot

//    /* Print the detection results. */
//    for (int i = 0; i < detections.size(); ++i) {
//        const vector<int>& d = detections[i];
//        // Detection format: [cls_id,left,right,top,bot]
//       cv::rectangle(img, cvPoint(d[1], d[3]), cvPoint(d[2],d[4]), cvScalar(255, 0, 0), 3, 8, 0);
//    }
//    cvNamedWindow("window");
//    cv::imshow("window",img);
//    cv::waitKey(0);
//-----------------------------------------Second Test----------------------------------------
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

//    //detector init
//    string model_file_ = "/home/onelly/tsq/caffe-yolov3-master/yolov3.prototxt";
//    string weights_file_ = "/home/onelly/tsq/caffe-yolov3-master/yolov3.caffemodel";

//    yolov3_detect *detector = new yolov3_detect(model_file_, weights_file_);
//    Caffe::set_mode(Caffe::GPU);

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
//                cv::Mat img(p_imageinfo->w,p_imageinfo->h,CV_8UC3);
//                img.data = p_imageinfo->data1;
//                img = img.reshape(0, p_imageinfo->h);

//                //process
//                std::vector<vector<int> > detections = detector->Detect(img,
//                    p_imageinfo->w,p_imageinfo->h,p_imageinfo->c,p_imageinfo->s);
//                //draw
//                for (int i = 0; i < detections.size(); ++i) {
//                    const vector<int>& d = detections[i];
//                    // Detection format: [cls_id,left,right,top,bot]
//                   cv::rectangle(img, cvPoint(d[1], d[3]), cvPoint(d[2],d[4]), cvScalar(255, 0, 0), 3, 8, 0);
//                }

//                //write
//                IplImage ipl = img;
//                p_imageinfo->h = ipl.height;
//                p_imageinfo->w = ipl.width;
//                p_imageinfo->c = ipl.nChannels;
//                p_imageinfo->s = ipl.widthStep;
//                img = img.reshape(0, 1);


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
//-----------------------------------------Third Test-----------------------------------------
//    printf("SHM_STRUCT size %ld\n",sizeof(SHM_STRUCT));
//    printf("IMAGEINFO size %ld\n",sizeof(IMAGEINFO));

//    key_t key;
//    IPC_shm ishm[SHM_QUEUE_NAME];

//    printf("attach shm start...%ld\n",time(0));
//    const char* name = key_path.c_str();
//    key = ftok(name,0);

//    ishm[0].siOpenShm(key);
//    ishm[0].siAttShm(NULL,0);
//    printf("name=%s,key=%d\n",name,key);
//    printf("attach shm ok...%ld\n",time(0));

//    int count = 0;
//    long framenum = 0;

//    //detector init
//    string model_file_ = "/home/onelly/model/Vel_Pes_Res18_M1_visp_deploy_bnmerge.prototxt";
//    string weights_file_ = "/home/onelly/model/1109_bias_bnmerge.caffemodel";
//    string mean_file_ = "";
//    string mean_value_ = "104,117,123";
//    Detector *detector = new Detector(model_file_, weights_file_, mean_file_, mean_value_);
//    Caffe::set_mode(Caffe::GPU);
//    Caffe::SetDevice(gpu_id);

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
//                cv::Mat img(p_imageinfo->w,p_imageinfo->h,CV_8UC3);
//                img.data = p_imageinfo->data1;
//                img = img.reshape(0, p_imageinfo->h);

//                //process
//                std::vector<vector<float> > detections = detector->Detect(img);
//                for (int i = 0; i < detections.size(); ++i) {
//                    const vector<float>& d = detections[i];
//                    // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
//                    //CHECK_EQ(d.size(), 7);
//                    const float score = d[2];
//                    if (score >= thresh)
//                    {
//                        string test;
//                        string label ="class :";
//                        label[5] = '0'+d[1];
//                        std::cout << label<<std::endl;
//                        test = std::to_string(score);
//                        test = label + test;
//                        cv::putText(img, test, cvPoint(static_cast<int>(d[3] * img.cols), static_cast<int>(d[4] * img.rows)), CV_FONT_HERSHEY_COMPLEX,0.8, CV_RGB(255, 0, 0),2,8,false);
//                        cv::rectangle(img, cvPoint(d[3] * img.cols, d[4] * img.rows), cvPoint(d[5] * img.cols, d[6] * img.rows), cvScalar(255, 0, 0), 2, 8, 0);
//                    }
//                }

//                //write
//                IplImage ipl = img;
//                p_imageinfo->h = ipl.height;
//                p_imageinfo->w = ipl.width;
//                p_imageinfo->c = ipl.nChannels;
//                p_imageinfo->s = ipl.widthStep;
//                img = img.reshape(0, 1);


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
//-----------------------------------------Fourth Test----------------------------------------
    printf("SHM_STRUCT size %ld\n",sizeof(SHM_STRUCT));
    printf("IMAGEINFO size %ld\n",sizeof(IMAGEINFO));

    key_t key;
    IPC_shm ishm[SHM_QUEUE_NAME];

    printf("attach shm start...%ld\n",time(0));
    const char* name = key_path.c_str();
    key = ftok(name,0);

    ishm[0].siOpenShm(key);
    ishm[0].siAttShm(NULL,0);
    printf("name=%s,key=%d\n",name,key);
    printf("attach shm ok...%ld\n",time(0));

    int count = 0;
    long framenum = 0;

    //detector init
    string model_file_ = model_file;
    string weights_file_ = weights_file;

    yolov3_detect *detector = new yolov3_detect(model_file_, weights_file_);
    Caffe::set_mode(Caffe::GPU);
    Caffe::SetDevice(gpu_id);

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
                std::vector<vector<int> > detections = detector->Detect(img,
                    p_imageinfo->w,p_imageinfo->h,p_imageinfo->c,p_imageinfo->s);
                //draw
                cv::Mat img2 = img.clone();
                if(detections.size()>0){
                    for (int x = 0; x < detections.size(); ++x) {
                        const vector<int>& d = detections[x];
                        // Detection format: [cls_id,left,right,top,bot]
                       cv::rectangle(img2, cvPoint(d[1], d[3]), cvPoint(d[2],d[4]), cvScalar(255, 0, 0), 3, 8, 0);
                    }

                    //write
                    IplImage ipl = img2;
                    p_imageinfo->h = ipl.height;
                    p_imageinfo->w = ipl.width;
                    p_imageinfo->c = ipl.nChannels;
                    p_imageinfo->s = ipl.widthStep;
                    img2 = img2.reshape(0, 1);
                    //write data
                    memcpy(p_imageinfo->data2,img2.data,p_imageinfo->h*p_imageinfo->w*p_imageinfo->c);
                    //write datalen2
                    p_imageinfo->datalen2 = 1;
                    //write recognition
                    char *dst = p_imageinfo->recognition;
                    sprintf(dst,"%s%s%s\0",p_imageinfo->year,p_imageinfo->month,p_imageinfo->day);
                    sprintf(dst,"%s-%d-%d-%d_%d\0",dst,p_imageinfo->areaid,p_imageinfo->batch,p_imageinfo->frames,gpu_id);
                    sprintf(dst,"%s|%d|\0",dst,detections.size());
                    sprintf(dst,"%s%d %d %d %d %d\0",dst,detections[0][0],detections[0][1],detections[0][3],detections[0][2],detections[0][4]);
                    for (int x = 1; x < detections.size(); ++x) {
                        const vector<int>& d = detections[x];
                        // Detection format: [cls_id,left,right,top,bot]
                        sprintf(dst,"%s|%d %d %d %d %d\0",dst,d[0],d[1],d[3],d[2],d[4]);
                    }
                    printf("recongnition:%s\n",p_imageinfo->recognition);
                    //write recongnition lenth
                    for(p_imageinfo->detlen=0;p_imageinfo->detlen<1024000;p_imageinfo->detlen++){
                        if(dst[p_imageinfo->detlen]==0)break;
                    }
                    //p_imageinfo->detlen++;
                    printf("detlen:%d\n",p_imageinfo->detlen);
                }
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


