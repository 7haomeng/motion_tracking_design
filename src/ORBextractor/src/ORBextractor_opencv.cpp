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

using namespace ros;
using namespace std;
// using namespace message_filters;
using namespace cv;


class ORB_extractor{
private:
    // ORBextractor* mpIniORBextractor;
    Mat RGB_frame;
    Mat mImGray;
    Mat outimg1;
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

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
    output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/image", 1);
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
    // keypoints_1.resize(100);

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    descriptor->compute(mImGray, keypoints_1, descriptors_1);
    drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar(0, 255, 255), DrawMatchesFlags::DEFAULT);

    for(unsigned int i = 0; i < keypoints_1.size(); i++){
        cout << "keypoints [" << i << "][x]" << ":" << keypoints_1[i].pt.x << endl;
        cout << "keypoints [" << i << "][y]" << ":" << keypoints_1[i].pt.y << endl;
        cout << endl;
    }

    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg1).toImageMsg();
    output_image_pub.publish(output_image);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "ORBextractor");
    ros::NodeHandle nh;
    ORB_extractor ORB_extractor(nh);
    ros::spin();
}