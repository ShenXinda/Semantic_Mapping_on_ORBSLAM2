
#ifndef GLOBAL_H
#define GLOBAL_H

#include <string>
#include <iostream> 

#include <opencv2/core/core.hpp>


// 此类保存所有从配置文件中（.yaml）的变量
namespace ORB_SLAM2
{

class Global 
{
public:
    Global(){}
    ~Global(){}
    
    void init(std::string strSettingsFile);

    // 因为类的声明并不会进行内存空间的分配，所以类的静态成员无法在类声明中定义。
    // 类的静态成员需要类内声明，类外定义。并且注意定义尽量不要出现在头文件中，以免造成重复定义。
    static int PORT;
    static int CHANNELS;
    static int HEIGHT;
    static int WIDTH;
    static std::string HOSTADDR;
};

}

#endif