#include "PointCloudMappingClient.h"
using namespace cv;

namespace ORB_SLAM2 {

PointCloudMappingClient::PointCloudMappingClient(string savePCDPath, double thprob, double thdepth): mCx(0), mCy(0), mFx(0), mFy(0), mbShutdown(false), mbFinish(false), mSavePCDPath(savePCDPath), mThdepth(thdepth), mThprob(thprob)
{
    this->initColorMap();
    mImgSeg.connectServer(); // 连接服务端

    mPointCloud = boost::make_shared<PointCloud>();  // 用boost::make_shared<> ?

    viewerThread = std::make_shared<std::thread>(&PointCloudMappingClient::showPointCloud, this); 
}

PointCloudMappingClient::~PointCloudMappingClient()
{
    viewerThread->join();
}

void PointCloudMappingClient::requestFinish()
{
    {
        unique_lock<mutex> locker(mKeyFrameMtx);
        mbShutdown = true;
    }
    mKeyFrameUpdatedCond.notify_one();
}

bool PointCloudMappingClient::isFinished()
{
    return mbFinish;
}

void PointCloudMappingClient::insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth)
{
    {
        unique_lock<mutex> locker(mKeyFrameMtx);
        mvKeyFrames.push(kf);
        mvColorImgs.push( color.clone() );  // clone()函数进行Mat类型的深拷贝，为什幺深拷贝？？
        mvDepthImgs.push( depth.clone() );
        cout << "receive a keyframe, id = " << kf->mnId << endl;
    }
    mKeyFrameUpdatedCond.notify_one();
    
}

void PointCloudMappingClient::showPointCloud() 
{
    pcl::visualization::CloudViewer viewer("Dense pointcloud viewer");
    while(true) {   
        KeyFrame* kf;
        cv::Mat colorImg, depthImg;

        {
            std::unique_lock<std::mutex> locker(mKeyFrameMtx);
            while(mvKeyFrames.empty() && !mbShutdown){  // !mbShutdown为了防止所有关键帧映射点云完成后进入无限等待
                mKeyFrameUpdatedCond.wait(locker); 
            }            
            
            if (!(mvDepthImgs.size() == mvColorImgs.size() && mvKeyFrames.size() == mvColorImgs.size())) {
                std::cout << "这是不应该出现的情况！" << std::endl;
                continue;
            }

            if (mbShutdown && mvColorImgs.empty() && mvDepthImgs.empty() && mvKeyFrames.empty()) {
                break;
            }

            kf = mvKeyFrames.front();
            colorImg = mvColorImgs.front();    
            depthImg = mvDepthImgs.front();    
            mvKeyFrames.pop();
            mvColorImgs.pop();
            mvDepthImgs.pop();
        }

        if (mCx==0 || mCy==0 || mFx==0 || mFy==0) {
            mCx = kf->cx;
            mCy = kf->cy;
            mFx = kf->fx;
            mFy = kf->fy;
        }

        
        generatePointCloud(colorImg, depthImg, kf->GetPose(), kf->mnId);
        viewer.showCloud(mPointCloud);
        
        std::cout << "show point cloud, size=" << mPointCloud->points.size() << std::endl;
    }

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize( 0.01, 0.01, 0.01); // 每1cm^3一个点
    PointCloud::Ptr tmp(new PointCloud);
    *tmp = *mPointCloud;
    voxel.setInputCloud(tmp);
    voxel.filter(*mPointCloud);
    // 存储点云
    pcl::io::savePCDFile(mSavePCDPath, *mPointCloud);
    cout << "save pcd files to :  " << mSavePCDPath << endl;
    mbFinish = true;
}

void PointCloudMappingClient::generatePointCloud(const cv::Mat& imRGB,  cv::Mat& imD, const cv::Mat& pose, int nId)
{ 
    std::cout << "Converting image: " << nId << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();     

    mImgSeg.handleImage(imRGB); // 将图像发送给服务端进行语义分割，并返回概率图
    vector<float> probMap = mImgSeg.getProbMap(); // 获取从服务端返回的概率图
    this->mergePointCloudFromImage(probMap, imRGB, imD, pose); // 合并每一帧图像生成的点云

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count(); 
    std::cout << "Cost = " << t << std::endl;
}

void PointCloudMappingClient::mergePointCloudFromImage(const vector<float>& probMap, const cv::Mat& imRGB, cv::Mat& imD, const cv::Mat& pose)
{   
    PointCloud::Ptr current(new PointCloud);

    Eigen::Isometry3d T = Converter::toSE3Quat( pose );

    if (mPointCloud->points.size() != 0) { // 地图中已经存在点，先进行关联
        assoiateAndUpdate(probMap, imD, pose);
    }

    for (int v = 1; v < Global::HEIGHT; v+=3) {
        for (int u = 1; u < Global::WIDTH; u+=3) {
            float d = imD.ptr<float>(v)[u];  // float类型！！！
            if(d <0.01 || d>10){ // 深度值测量失败（或者是已经匹配过的像素）
                continue;
            }

            PointT p;
            p.z = d;
            p.x = ( u - mCx) * p.z / mFx;
            p.y = ( v - mCy) * p.z / mFy;

            // 根据深度距离给颜色，越远颜色越淡
            p.b = d/10*255;
            p.g = d/10*255;
            p.r = d/10*255;
            // p.b = imRGB.ptr<uchar>(v)[u*3];
            // p.g = imRGB.ptr<uchar>(v)[u*3+1];
            // p.r = imRGB.ptr<uchar>(v)[u*3+2];
            current->points.push_back(p);
        }
    }

    PointCloud::Ptr tmp(new PointCloud);
    pcl::transformPointCloud(*current, *tmp, T.inverse().matrix()); 
    *current = *tmp;
    // pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    // statistical_filter.setMeanK(50);
    // statistical_filter.setStddevMulThresh(1.0); // The distance threshold will be equal to: mean + stddev_mult * stddev
    // statistical_filter.setInputCloud(tmp);
    // statistical_filter.filter(*current);
    // current->is_dense = false; 
    int k = mPointCloud->points.size();

    for (size_t i = 0;  i < current->points.size(); i++) {  // 初始化点概率均匀分布
        mvPoint2ProbPoint[k++] = new ProbPoint(Global::CHANNELS);
    }

    *mPointCloud += *current;
}

// 二维图像中像素点与三维点云中的地图点关联
void PointCloudMappingClient::assoiateAndUpdate(const vector<float>& probMap, cv::Mat& imD, const cv::Mat& pose)
{
    set<int> labels;
    Eigen::Isometry3d T = Converter::toSE3Quat( pose ); 
    for (size_t i = 0; i < mPointCloud->points.size(); i++) {
        Eigen::Vector3d pw;
        pw[0] = mPointCloud->points[i].x;
        pw[1] = mPointCloud->points[i].y;
        pw[2] = mPointCloud->points[i].z;
        
        Eigen::Vector3d pc = T * pw;   // 注意T不用取逆（pose与离线建图时直接读取的不同）

        // 重投影像素坐标[u,v]
        double u = mFx*(pc[0]/pc[2]) + mCx;
        double v = mFy*(pc[1]/pc[2]) + mCy;

        if (u < 0 || u >= Global::WIDTH || v < 0 || v >= Global::HEIGHT) {
            continue;
        }

        int uleft = floor(u), uright = ceil(u), vup = floor(v), vdown = ceil(v);
        vector<pair<int,int>> candidatePoint = {pair<int,int>(uleft, vup), 
            pair<int,int>(uleft, vdown), pair<int,int>(uright, vup), pair<int,int>(uright, vdown)};  // 重投影点周围的四个点

        double minValue = 10.0;
        int w = -1, h = -1;
        for (size_t k = 0;  k < candidatePoint.size(); k++) {
            int row = candidatePoint[k].second, col = candidatePoint[k].first;
            if (row >= imD.rows || col >= imD.cols) {  // 防止越界（图像是一维存储的，只有行和列的乘积大于了总的元素，程序才会错误；但是行和列超出了边界，读取的数据是错的）
                continue;
            }
            float d = imD.ptr<float>(row)[col];
            if(d <0.01 || d>10){ // 深度值测量失败
                continue;
            }
            if (fabs(pc[2]-d) < minValue) {
                minValue = fabs(pc[2]-d);
                h = row;
                w = col;
            }
        }

        // mThdepth为深度误差阈值
        if (minValue>mThdepth || w==-1 || h==-1) {
            continue;
        }
        imD.ptr<float>(h)[w] = 0;  // 已经匹配过的像素深度设置为0

        ProbPoint* probPoint = mvPoint2ProbPoint[i]; 

        vector<float> vNewProbs(Global::CHANNELS);
        for (int c = 0; c < Global::CHANNELS; c++) {
            int index = c*Global::HEIGHT*Global::WIDTH + h*Global::WIDTH + w;
            vNewProbs[c] = probMap[index];  
        }
        
        int label = probPoint->BayesianUpdate(vNewProbs);

        // mThprob为最小概率阈值
        if (label == -1 || (probPoint->getProbVector())[label] < mThprob) {
            continue;
        }
        
        label = mergeSomeClasses(label);
        
        if (colorMap.find(label) != colorMap.end() && std::find(things.begin(), things.end(), colorMap[label].second) != things.end()) {
            labels.insert(label);
            mPointCloud->points[i].b = colorMap[label].first[2];
            mPointCloud->points[i].g = colorMap[label].first[1];
            mPointCloud->points[i].r = colorMap[label].first[0];
        }
    }
    cout << "The indentified Categories: " ;
    for (int la: labels) {
        cout << colorMap[la].second << " ";
    }
    cout << endl;
}

// 加载colormap.txt和things.txt文件，与语义标签显示相关
void PointCloudMappingClient::initColorMap()
{
    // fr1_desk1,2  0,46,61,71,72,73,75,76,83,109,117,174(wall)
    // things = {"penson" , "cup", "chair", "tv", "laptop", "mouse", "keyboard", "cell-phone", "book", "desk","floor","wall"}; 

    // fr2_desk 0,46,61,63,71,73,75,83,87,109,117,174(wall)
    // things = {"person", "cup", "chair", "potted-plant", "tv", "mouse", "keyboard", "book","teddy-bear","desk","floor"};  // with stuff
    // things = {"person", "cup", "chair", "potted-plant", "tv", "mouse", "keyboard", "book","teddy-bear"}; 


    // lab_20 36,46,61,71,73,74,75,76,83,109,117,174(wall)
    // things = {"sports-ball", "cup", "chair", "tv", "mouse","remote", "keyboard","cell-phone","book","desk","floor","wall"};  // with stuff
    // things = {"sports-ball", "cup", "chair", "tv", "mouse","remote","keyboard","cell-phone","book"};

    // office_desk_20 36,43,46,51,52,54,61,63,71,72,73,74,75,76,83,109,117,174(wall)
    // things = {"sports-ball", "bottle","cup", "banana","apple","orange","chair", "potted-plant","tv", "mouse","remote","keyboard","cell-phone","book","desk","floor","wall"};

    std::ifstream fthings;
    string path = "./things.txt";  // 保存了需要识别的语义标签，每行为[name]。
    fthings.open(path.c_str());
    while(!fthings.eof()) {
        string s;
        getline(fthings,s);
        things.push_back(s);
    }
    fthings.close();

    std::ifstream fcolormap; 
    path = "./colormap.txt";  // 保存了所有类别的RGB值，每行为[id name r g b]。
    fcolormap.open(path.c_str());
    //C++ eof()函数可以帮助我们用来判断文件是否为空，或是判断其是否读到文件结尾。
    while(!fcolormap.eof()) {
        string s;
        getline(fcolormap,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            int id, r, g, b;
            string name;
            ss >> id >> name >> r >> g >> b;
            colorMap[id] = std::pair<std::vector<int>, std::string>({r,g,b}, name);
        }
    }
    fcolormap.close();

}

// 对coco-stuff数据集的182类中相似的类别进行合并
int PointCloudMappingClient::mergeSomeClasses(int label) 
{
    // floor相关的统一归为floor
    if ((label>=113 && label<=117) || label==139) {
        label = 117;
    }
    // desk、table相关的统一归为desk
    if (label == 66 || label==68 || label==109 || label==164) {
        label = 109;
    }
    // wall相关的统一归类为wall
    if (label>=170 && label<=176) {
        label = 174;
    }

    return label;
}


}