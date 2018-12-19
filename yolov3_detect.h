#ifndef YOLOV3_DETECT_H
#define YOLOV3_DETECT_H
#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <stdio.h>
//image
#include <image.h>

using namespace caffe;

class yolov3_detect
{
public:
    yolov3_detect(const string& model_file, const string& weights_file );
    shared_ptr<Net<float> > net;
    std::vector<vector<int> > Detect(cv::Mat mat,int w, int h, int c, int step);
    int size;
    image Mat2Image(cv::Mat inputMat,int w,int h, int c, int step);
    cv::Mat Image2Mat(image inputImage);

    image make_empty_image(int w, int h, int c);
    image make_image(int w, int h, int c);
//    uint64_t current_timestamp();
//    bool signal_recieved = false;
//    void sig_handler(int signo);
};

#endif // YOLOV3_DETECT_H
