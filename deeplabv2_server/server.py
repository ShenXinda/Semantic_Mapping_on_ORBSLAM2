# https://www.pythonf.cn/read/57971

# socket()->bind()->listen()->accept()->recv()-send()

#!/usr/bin/env python
# coding=utf-8
from socket import  *
import cv2
import matplotlib.pyplot as plt
import numpy as np
import inference
# from signal import signal, SIGPIPE, SIG_DFL

# # 让 python 忽略 SIGPIPE 信号，并且不抛出异常
# signal(SIGPIPE,SIG_DFL) 


HOST = '127.0.0.1'
PORT = 8080
ADDR = (HOST,PORT)

def recvall(sock, count):
        buf = b''  # buf是一个byte类型
        while count:
            # 接受TCP套接字的数据。数据以字符串形式返回，count指定要接收的最大数据量.
            newbuf = sock.recv(count)
            if not newbuf: 
                return None
            buf += newbuf
            count -= len(newbuf)
            print(count)
        return buf


# 语义分割模型初始化
inference.init()

# 使用 with as 语句操作上下文管理器（context manager），它能够帮助我们自动分配并且释放资源。
# 使用 with as 操作已经打开的文件对象，无论期间是否抛出异常，都能保证 with as 语句执行完毕后自动关闭已经打开的文件。
# 创建AF_INET地址族，TCP的套接字
with socket(AF_INET,SOCK_STREAM) as tcpSerSock:
    #绑定ip和端口
    tcpSerSock.bind(ADDR)
    #监听端口，是否有请求
    tcpSerSock.listen(5)

    print("Listen on port",PORT)
    
    while True:
        
        # accept() 是阻塞的
        tcpClientSock,addr = tcpSerSock.accept()
        print("The client: ",addr,"is connecting.")

        with tcpClientSock:
            # 使用一个while循环，持续和客户端通信，直到客户端断开连接或者崩溃
            while True:                
                # Bytes 对象是由单个字节作为基本元素（8位，取值范围0-255）组成的序列，为不可变对象。 
                # Bytes 对象只负责以二进制字节序列的形式记录所需记录的对象，至于该对象到底表示什么（比如到底是什么字符）则由相应的编码格式解码所决定。
                count = recvall(tcpClientSock,16)
                if not count:
                    break

                print("The file size: ",(int)(count))
                stringData = recvall(tcpClientSock,(int)(count))
                if not stringData:
                    break

                data = np.frombuffer(stringData, np.uint8)
                img = cv2.imdecode(data, cv2.IMREAD_COLOR)  # 将数组解码成图像
                # img = cv2.imdecode(np.asarray(bytearray(stringData),dtype='uint8'), cv2.IMREAD_COLOR)
                print(img.shape)
                probs = inference.runSegModel(img)
                print(type(probs))  # <class 'numpy.ndarray'>
                print(probs.shape)
                print(probs.dtype)  # numpy中元素类型为float32
                # print(np.max(probs,0))

                stringProbs = probs.tostring()
                # 注意是客户端的套接字
                tcpClientSock.sendall(str.encode(str(len(stringProbs)).ljust(16)))
                tcpClientSock.sendall(stringProbs)
            
            #客户端退出
            print("client ",addr,"is disconnected.")