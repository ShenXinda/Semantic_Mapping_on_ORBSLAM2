#include "Global.h"

namespace ORB_SLAM2
{

// 定义的时候不用static修饰,static在类外面表示限定作用域（放在头文件会出现多重定义）
int Global::PORT = 8080;
int Global::CHANNELS = 182;
int Global::HEIGHT = 480;
int Global::WIDTH = 640;
std::string Global::HOSTADDR = "127.0.0.1";

void Global::init(std::string strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
       exit(-1);
    }
    WIDTH = fsSettings["Mapping.WIDTH"];
    HEIGHT = fsSettings["Mapping.HEIGHT"];
    CHANNELS = fsSettings["Mapping.CHANNELS"];
    PORT = fsSettings["Mapping.PORT"];
    std::string hostAddr = fsSettings["Mapping.HOSTADDR"]; 
    HOSTADDR = hostAddr;
}
    
} // namespace ORB_SLAM2

