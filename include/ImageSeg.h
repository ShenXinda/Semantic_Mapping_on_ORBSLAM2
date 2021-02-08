
#ifndef IMAGESEG_H
#define IMAGESEG_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Global.h"

using namespace std;



namespace ORB_SLAM2 
{

class Global;
const int BUFFERSIZE = 1<<30;

class ImageSeg 
{
public:
    ImageSeg();
    ~ImageSeg();
    void closeClientSockfd();
    bool connectServer();
    void handleImage(const cv::Mat& imRGB);
    const vector<float>& getProbMap();
    

private:
    int transMatToCharArray(char* modelImage, const cv::Mat& imRGB);
    bool isRecvSuccess(int len);
    
    int mClientSockfd;
    vector<float> mProbMap;
    bool mIsFirstImage;
};
    
}







#endif