
#include "ImageSeg.h"
#include "Global.h"
using namespace std;
using namespace cv;

int main() {
    ORB_SLAM2::ImageSeg imgSeg;
    imgSeg.connectServer();

    while(1) {
        string fileName = "";
        cout << "Place enter the image name: " << endl;
        cin >> fileName;
        Mat img = imread(fileName);
        imgSeg.handleImage(img);
        vector<float> probMap = imgSeg.getProbMap();
        for (int c = 0; c < ORB_SLAM2::Global::CHANNELS; c++) {
            float m = 0;
            for (int h = 0; h < ORB_SLAM2::Global::HEIGHT; h++) {
                for (int w = 0; w < ORB_SLAM2::Global::WIDTH; w++) {
                    int index = c*ORB_SLAM2::Global::HEIGHT*ORB_SLAM2::Global::WIDTH + h*ORB_SLAM2::Global::WIDTH + w;
                    float prob = probMap[index];
                    m = max(m, prob);
                }
            }
            cout << c << ": " << m << endl; 
        }
    }

    imgSeg.closeClientSockfd();
}