#include "ImageSeg.h"

namespace ORB_SLAM2 
{

// public
ImageSeg::ImageSeg():mClientSockfd(-1), mIsFirstImage(true){}

ImageSeg::~ImageSeg(){
    this->closeClientSockfd();
}

bool ImageSeg::connectServer()
{
    int client_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_sockfd == -1) {
        perror("Socket error");
        return false;
    }
    
    struct sockaddr_in client_addr;
    bzero(&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(Global::PORT);
    inet_pton(AF_INET,Global::HOSTADDR.c_str(), &client_addr.sin_addr.s_addr);
    
    int ret = connect(client_sockfd, (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (ret == -1) {
        perror("Connect error");
        return false;
    }

    char ipbuf[128];
    printf("Connect successfully! client iP: %s, port: %d\n", inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, ipbuf, 
        sizeof(ipbuf)), ntohs(client_addr.sin_port));

    mClientSockfd = client_sockfd;
    return true;
}

void ImageSeg::handleImage(const cv::Mat& imRGB)
{
    

    char* buf = new char[BUFFERSIZE];
    int bufLength = this->transMatToCharArray(buf, imRGB); 
    cout << bufLength << endl;

    // int转char数组
    char lenStrSend[16];
    stringstream ss;
    ss << bufLength;
    ss >> lenStrSend;
    for (int i = strlen(lenStrSend); i < sizeof(lenStrSend) ; i++) {
        lenStrSend[i] = ' ';
    }
    send(mClientSockfd, lenStrSend, sizeof(lenStrSend), 0);
    send(mClientSockfd, buf, bufLength, 0);
    delete [] buf;

    char lenStrRec[16];
    // 阻塞读取
    int len = recv(mClientSockfd, lenStrRec, sizeof(lenStrRec), 0);
    if (!isRecvSuccess(len)) {
        return;
    }

    int size;
    stringstream stream(lenStrRec);  
    stream >> size; 
    char* recBuf = new char[BUFFERSIZE]; 
    len = 0;
    do {
        int l = recv(mClientSockfd, recBuf+len, BUFFERSIZE-len, 0);
        if (!isRecvSuccess(l)) {
            return;
        }
        len += l;
    }while(len < size);

    for (int c = 0; c < Global::CHANNELS; c++) {
        for (int h = 0; h < Global::HEIGHT; h++) {
            for (int w = 0; w < Global::WIDTH; w++) {
                int index = c*Global::HEIGHT*Global::WIDTH + h*Global::WIDTH + w;
                char c_heading[4];
                for (int i = 0; i < 4; i++) {   // float四个字节
                    c_heading[i] = recBuf[index*4+i];
                }
                if (mIsFirstImage) {
                    mProbMap.push_back(*((float*)(c_heading))); 
                }else {
                    mProbMap[index] = *((float*)(c_heading));
                }
                
            }
        }
    }
    mIsFirstImage = false;

    delete [] recBuf; 
}

void ImageSeg::closeClientSockfd()
{
    if (mClientSockfd != -1) {
        close(mClientSockfd);
        mClientSockfd = -1;
    }
}

const vector<float>& ImageSeg::getProbMap()
{
    return mProbMap;
}

// private

bool ImageSeg::isRecvSuccess(int len)
{
    if (len == -1) {
        perror("Read error");
    }else if (len == 0){
        printf("Without receiving data's Length from server.");
    }else{
        return true;
    }
    this->closeClientSockfd();
    return false;
}

int ImageSeg::transMatToCharArray(char* modelImage, const cv::Mat& imRGB)
{
    // 将cv::Mat数据编码成数据流
    std::vector<uchar> buff;
    cv::imencode(".png", imRGB, buff);
    memset(modelImage, 0, BUFFERSIZE);
    memcpy(modelImage, reinterpret_cast<char*>(&buff[0]), buff.size());
    return buff.size();
}

}

