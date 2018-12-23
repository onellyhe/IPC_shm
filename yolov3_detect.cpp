//#include "yolov3_detect.h"
//#include <caffe/caffe.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <algorithm>
//#include <iomanip>
//#include <iosfwd>
//#include <memory>
//#include <string>
//#include <utility>
//#include <vector>
//#include <sys/time.h>
//#include <pthread.h>
//#include <stdio.h>

//#include <activations.h>
//#include <blas.h>
//#include <box.h>

//#include <yolo_layer.h>
//#include <./cuda.h>
//#include <image.h>


////#define CPU_ONLY
//using namespace caffe;
//using namespace cv;



//yolov3_detect::yolov3_detect(const string& model_file, const string& weights_file)
//{
//    //定义工作模式CPU or GPU
//#ifdef CPU_ONLY
//    Caffe::set_mode(Caffe::CPU);
//#else
//    Caffe::set_mode(Caffe::GPU);
//#endif
//    /* Load the network. */
//    net.reset(new Net<float>(model_file, TEST));
//    net->CopyTrainedLayersFrom(weights_file);

//    printf("num_inputs is %d\n",net->num_inputs());
//    printf("num_outputs is %d\n",net->num_outputs());
//    CHECK_EQ(net->num_inputs(), 1) << "Network should have exactly one input.";
//    CHECK_EQ(net->num_outputs(), 3) << "Network should have exactly three outputs.";


//}

//cv::Mat yolov3_detect::Image2Mat(image inputImage){//may not use
//    cv::Mat outputMat(inputImage.w,inputImage.h,CV_8UC3);
//    //TODO
//    return outputMat;
//}

//image yolov3_detect::Mat2Image(cv::Mat inputMat, int w, int h, int c,int step){
//    image outputImage = make_image(w, h, c);
//    uchar* data = inputMat.data;
//    int i, j, k;
//    for(i = 0; i < h; ++i){
//        for(k= 0; k < c; ++k){
//            for(j = 0; j < w; ++j){
//                outputImage.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
//            }
//        }
//    }
//    return outputImage;
//}

//std::vector<vector<int> > yolov3_detect::Detect(cv::Mat input,int w,int h, int c, int step){



//    Blob<float> *input_data_blobs = net->input_blobs()[0];
//    LOG(INFO) << "Input data layer channels is  " << input_data_blobs->channels();
//    LOG(INFO) << "Input data layer width is  " << input_data_blobs->width();
//    LOG(INFO) << "Input data layer height is  " << input_data_blobs->height();

//    size = input_data_blobs->channels()*input_data_blobs->width()*input_data_blobs->height();

//    //load image
//    image im = Mat2Image(input,w,h,c,step);
//    image sized = letterbox_image(im,input_data_blobs->width(),input_data_blobs->height());
//    cuda_push_array(input_data_blobs->mutable_gpu_data(),sized.data,size);

//    net->Forward();

//    vector<Blob<float>*> blobs;
//    blobs.clear();
//    Blob<float>* out_blob1 = net->output_blobs()[1];
//    blobs.push_back(out_blob1);
//    Blob<float>* out_blob2 = net->output_blobs()[2];
//    blobs.push_back(out_blob2);
//    Blob<float>* out_blob3 = net->output_blobs()[0];
//    blobs.push_back(out_blob3);

//    printf("output blob1 shape c= %d, h = %d, w = %d\n",out_blob1->channels(),out_blob1->height(),out_blob1->width());
//    printf("output blob2 shape c= %d, h = %d, w = %d\n",out_blob2->channels(),out_blob2->height(),out_blob2->width());
//    printf("output blob3 shape c= %d, h = %d, w = %d\n",out_blob3->channels(),out_blob3->height(),out_blob3->width());

//    int nboxes = 0;
//    detection *dets = get_detections(blobs,im.w,im.h,&nboxes);

//    //        uint64_t endDetectTime = current_timestamp();
//    //        printf("object-detection:  finished processing data operation  (%zu)ms\n", endDataTime - beginDataTime);
//    //        printf("object-detection:  finished processing yolov3 network  (%zu)ms\n", endDetectTime - beginDetectTime);

//    //show detection results
//    std::vector<vector<int> > detections;

//    int i,j;
//    for(i=0;i< nboxes;++i){
//        int cls = -1;
//        for(j=0;j<80;++j){
//            if(dets[i].prob[j] > 0.5){
//                if(cls < 0){
//                    cls = j;
//                }
//                printf("%d: %.0f%%\n",cls,dets[i].prob[j]*100);
//            }
//        }
//        if(cls >= 0){
//            box b = dets[i].bbox;
//            printf("x = %f,y =  %f,w = %f,h =  %f\n",b.x,b.y,b.w,b.h);

//            int left  = (b.x-b.w/2.)*im.w;
//            int right = (b.x+b.w/2.)*im.w;
//            int top   = (b.y-b.h/2.)*im.h;
//            int bot   = (b.y+b.h/2.)*im.h;
//            vector<int> detection_of_obj;
//            detection_of_obj.push_back(cls);
//            detection_of_obj.push_back(left);
//            detection_of_obj.push_back(right);
//            detection_of_obj.push_back(top);
//            detection_of_obj.push_back(bot);
//            detections.push_back(detection_of_obj);
////            printf("left = %d,right =  %d,top = %d,bot =  %d\n",left,right,top,bot);
//        }
//    }



//    free_detections(dets,nboxes);
//    free_image(im);
//    free_image(sized);

//    printf("detectnet-camera:  video device has been un-initialized.\n");
//    printf("detectnet-camera:  this concludes the test of the video device.\n");
//    return detections;
//}


//image yolov3_detect::make_empty_image(int w, int h, int c)
//{
//    image out;
//    out.data = 0;
//    out.h = h;
//    out.w = w;
//    out.c = c;
//    return out;
//}

//image yolov3_detect::make_image(int w, int h, int c)
//{
//    image out = make_empty_image(w,h,c);
//    out.data = (float*)calloc(h*w*c, sizeof(float));
//    return out;
//}

