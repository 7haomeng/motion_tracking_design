#include <iostream>
#include <cstdio>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <list>
#include <opencv/cv.h>

// #include "../include/ORBextractor.h"
// #include <ORBextractor.h>

using namespace ros;
using namespace std;
// using namespace message_filters;
using namespace cv;

// #ifndef ORBEXTRACTOR_H
// #define ORBEXTRACTOR_H




// namespace ORB_SLAM2
// {

// class ExtractorNode
// {
// public:
//     ExtractorNode():bNoMore(false){}

//     void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

//     std::vector<cv::KeyPoint> vKeys;
//     cv::Point2i UL, UR, BL, BR;
//     std::list<ExtractorNode>::iterator lit;
//     bool bNoMore;
// };

// class ORB_extractor
// {
// private:
//     // ORBextractor* mpIniORBextractor;
//     Mat RGB_frame;
//     Mat mImGray;

// public:

//     ORB_extractor(ros::NodeHandle);
//     ~ORB_extractor();
//     void RGB_img_Callback(const sensor_msgs::ImageConstPtr&);

//     ros::Subscriber RGB_img_sub;
    
//     enum {HARRIS_SCORE=0, FAST_SCORE=1 };

//     void ORBextractor(int nfeatures, float scaleFactor, int nlevels,
//                  int iniThFAST, int minThFAST);

//     // ~ORBextractor(){}

//     // Compute the ORB features and descriptors on an image.
//     // ORB are dispersed on the image using an octree.
//     // Mask is ignored in the current implementation.
//     void operator()( cv::InputArray image, cv::InputArray mask,
//       std::vector<cv::KeyPoint>& keypoints,
//       cv::OutputArray descriptors);

//     int inline GetLevels(){
//         return nlevels;}

//     float inline GetScaleFactor(){
//         return scaleFactor;}

//     std::vector<float> inline GetScaleFactors(){
//         return mvScaleFactor;
//     }

//     std::vector<float> inline GetInverseScaleFactors(){
//         return mvInvScaleFactor;
//     }

//     std::vector<float> inline GetScaleSigmaSquares(){
//         return mvLevelSigma2;
//     }

//     std::vector<float> inline GetInverseScaleSigmaSquares(){
//         return mvInvLevelSigma2;
//     }

//     std::vector<cv::Mat> mvImagePyramid;

// protected:

//     void ComputePyramid(cv::Mat image);
//     void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
//     std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
//                                            const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

//     void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
//     std::vector<cv::Point> pattern;

//     int nfeatures;
//     double scaleFactor;
//     int nlevels;
//     int iniThFAST;
//     int minThFAST;

//     std::vector<int> mnFeaturesPerLevel;

//     std::vector<int> umax;

//     std::vector<float> mvScaleFactor;
//     std::vector<float> mvInvScaleFactor;    
//     std::vector<float> mvLevelSigma2;
//     std::vector<float> mvInvLevelSigma2;
// };

// } //namespace ORB_SLAM

// #endif

class ORB_extractor{
private:
    // ORBextractor* mpIniORBextractor;
    Mat RGB_frame;
    Mat mImGray;
    Mat outimg1;

    sensor_msgs::ImagePtr output_image;

public:
    ORB_extractor(ros::NodeHandle);
    ~ORB_extractor();
    void RGB_img_Callback(const sensor_msgs::ImageConstPtr&);

    ros::Subscriber RGB_img_sub;
    ros::Publisher output_image_pub;
};

ORB_extractor::ORB_extractor(ros::NodeHandle nh){
    RGB_img_sub = nh.subscribe("/camera/color/image_raw", 1, &ORB_extractor::RGB_img_Callback,this);
    output_image_pub = nh.advertise<sensor_msgs::Image>("ORBextractor/ORBextractor_image", 1);
}

ORB_extractor::~ORB_extractor(){
}

void ORB_extractor::RGB_img_Callback(const sensor_msgs::ImageConstPtr& RGBimg_msg){
    RGB_frame = cv_bridge::toCvCopy(RGBimg_msg, "bgr8")->image;
    assert(RGB_frame.data != nullptr);
    mImGray = RGB_frame;
    cvtColor(mImGray, mImGray, CV_RGB2GRAY);

    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    keypoints_1.resize(100);
    // try{
    //     Ptr<FeatureDetector> detector = ORB::create();
    //     Ptr<DescriptorExtractor> descriptor = ORB::create();
    //     detector->detect(mImGray, keypoints_1);
    // }catch(cv::Exception& e){
    //     const char* err_msg = e.what();
    //     cout << "Exception caught: " << err_msg << endl;
    // }
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    // descriptor->compute(mImGray, keypoints_1, descriptors_1);
    // drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg1).toImageMsg();
    // output_image_pub.publish(output_image);
    // imshow("Opencv Extract ORB", outimg1);
    // waitKey(0);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "ORBextractor");
    ros::NodeHandle nh;
    ORB_extractor ORB_extractor(nh);
    ros::spin();
}