
#ifndef POINTCLOUDMAPPINGCLIENT_H
#define POINTCLOUDMAPPINGCLIENT_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <map>
#include <unordered_map>
#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>  
#include <Eigen/Geometry> 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "KeyFrame.h"
#include "Converter.h"
#include "ProbPoint.h"
#include "ImageSeg.h"
#include "Global.h"


typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef pcl::PointXYZRGBA PointT; // A point structure representing Euclidean xyz coordinates, and the RGB color.
typedef pcl::PointCloud<PointT> PointCloud;

namespace ORB_SLAM2 {

class Converter;
class KeyFrame;
class ProbPoint;
class ImageSeg;
class Global;

class PointCloudMappingClient {
    public:
        PointCloudMappingClient(string savePCDPath, double thprob=0.95, double thdepth=0.02);
        ~PointCloudMappingClient();
        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth); // 传入的深度图像的深度值单位已经是m
        void requestFinish();
        bool isFinished();

    private:
        void showPointCloud();

        void initColorMap();
        void runSegmantation();
        void mergePointCloudFromImage(const vector<float>& probMap, const cv::Mat& imRGB, cv::Mat& imD, const cv::Mat& pose);
        void assoiateAndUpdate(const vector<float>& probMap, cv::Mat& imD, const cv::Mat& pose);
        int mergeSomeClasses(int label);

        void generatePointCloud(const cv::Mat& imRGB, cv::Mat& imD, const cv::Mat& pose, int nId); 
        

        double mCx, mCy, mFx, mFy;
        
        std::shared_ptr<std::thread>  viewerThread;
  
        std::mutex mKeyFrameMtx;
        std::condition_variable mKeyFrameUpdatedCond;
        std::queue<KeyFrame*> mvKeyFrames;
        std::queue<cv::Mat> mvColorImgs, mvDepthImgs;

        PointCloud::Ptr mPointCloud;

        bool mbShutdown;
        bool mbFinish;

        ImageSeg mImgSeg;
        std::unordered_map<int,std::pair<std::vector<int>, std::string>> colorMap;
        std::vector<std::string> things;

        std::map<int, ProbPoint*> mvPoint2ProbPoint;

        string mSavePCDPath;
        double mThdepth;
        double mThprob;
};

}

#endif