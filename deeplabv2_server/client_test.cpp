// client 端相对简单, 另外可以使用nc命令连接->nc ip prot
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
using namespace std;


const int port = 8080;
const int buffer_size = 1<<31-1;
const int CHANNELS = 182;
const int HEIGHT = 480;
const int WIDTH = 640;

int main(int argc, const char *argv[]) {

    int client_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_sockfd == -1) {
        perror("socket error");
        exit(-1);
    }
    

    struct sockaddr_in client_addr;
    bzero(&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &client_addr.sin_addr.s_addr);
    
    int ret = connect(client_sockfd, (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (ret == -1) {
        perror("connect error");
        exit(-1);
    }

    char ipbuf[128];
    printf("Connect successfully! client iP: %s, port: %d\n", inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, ipbuf, 
        sizeof(ipbuf)), ntohs(client_addr.sin_port));
    // printf("Please input string to convert.\n");

    
    while(1) {
        // char buf[buffer_size] = {0};
        // fgets(buf, sizeof(buf), stdin); // 从终端读取字符串

        string imageName;
        cout << "Please enter Image name: " << endl;
        cin >> imageName;
        // 1. 打开图片文件
        ifstream is(imageName, ifstream::in);
        // 2. 计算图片长度length
        is.seekg(0, is.end);
        int length = is.tellg();

        // int转char数组
        char lenStr[16];
        stringstream ss;
        ss << length;
        ss >> lenStr;
        for (size_t i = strlen(lenStr); i < sizeof(lenStr) ; i++) {
            lenStr[i] = ' ';
        }
        send(client_sockfd, lenStr, sizeof(lenStr), 0);

        is.seekg(0, is.beg);
        // 3. 创建内存缓存区
        char* buf = new char[length];
        // 4. 读取图片
        is.read(buf, length);
        // 到此，图片已经成功的被读取到内存（buffer）中   
        send(client_sockfd, buf, length, 0);  
        delete [] buf; 

        char lenStr2[16];
        // read data, 阻塞读取
        int len = recv(client_sockfd, lenStr2, sizeof(lenStr2), 0);
        if (len == -1) {
            close(client_sockfd);
            perror("read error");
            return -1;
        }else if(len == 0){
            break;
        }

        int size;
        stringstream stream(lenStr2);  
        stream >> size; 

        char* recBuf = new char[buffer_size]; 
        len = 0;
        do {
            int l = recv(client_sockfd, recBuf+len, buffer_size-len, 0);
            if (l == -1) {
                close(client_sockfd);
                perror("read error");
                return -1;
            }else if(l == 0){
                break;
            }
            len += l;
            cout << len << endl;
        }while(len < size);
        
        if (len == -1) {
            close(client_sockfd);
            perror("read error");
            return -1;
        }else if(len == 0){
            break;
        }


        
        for (int c = 0; c < CHANNELS; c++) {
            float m = 0;
            for (int h = 0; h < HEIGHT; h++) {
                for (int w = 0; w < WIDTH; w++) {
                    int index = (c*HEIGHT*WIDTH + h*WIDTH + w)*4;
                    float prob;
                    char c_heading[4];
                    for (int i = 0; i < 4; i++) {
                        c_heading[i] = recBuf[index+i];
                    }
                    prob = *((float*)(c_heading));  // float四个字节  
                    m = max(m, prob);
                }
            }
            cout << c << ": " << m << endl; 
        }

        delete [] recBuf; 
    }
    
    close(client_sockfd);
    return 0;
}