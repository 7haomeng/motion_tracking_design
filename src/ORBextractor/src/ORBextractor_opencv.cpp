#include <iostream>
#include <cstdio>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point32.h>
#include "LKOpticalFlow_msgs/Points.h"


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <vector>
#include <list>
#include <opencv/cv.h>

using namespace ros;
using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, LKOpticalFlow_msgs::Points> OutlierRemovalSyncPolicy;
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
typedef message_filters::Synchronizer<OutlierRemovalSyncPolicy> OutlierRemovalSynchronizer;

class ORB_extractor{
private:
    // ros::NodeHandle tracking_nh;
    unsigned int i_;
    int cnt = 0;
    int r , c;
    int index = 0,orb_index = 0;
    int index_x = 0,index_orb_x = 0,index_y = 0,index_orb_y = 0;
    int keypoints_x,keypoints_y;
    int mask_label;
    int channels,orb_channels;
    float tst = 0;
    bool check = false;
    int optical_color_b;
    int optical_color_g;
    int optical_color_r;
    int color_b;
    int color_g;
    int color_r;
    size_t rows, cols,orb_rows, orb_cols;
    uchar* data;
    Mat result;
    Mat mask_img;
    Mat motion_img;
    Mat rs_img,predict_frame;
    Mat mImGray,predict_mImGray;
    Mat outimg1,predict_outimg,outimg2;
    Mat element;
    vector<KeyPoint> keypoints_1, predict_keypoints_1, keypoints_2;
    Point2f outlier_points;
    Mat descriptors_1, predict_descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    vector<Point2f> pt_vector;

    sensor_msgs::ImagePtr output_image;
    sensor_msgs::ImagePtr predict_image;
    sensor_msgs::ImagePtr predict_output_image;
    sensor_msgs::ImagePtr remove_outlier_image;
    geometry_msgs::Point32 points_msgs;

public:
    ORB_extractor(ros::NodeHandle);
    ~ORB_extractor();

    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    message_filters::Subscriber<sensor_msgs::Image> mask_sub;
    message_filters::Subscriber<sensor_msgs::Image> motion_sub;
    message_filters::Subscriber<LKOpticalFlow_msgs::Points> tracking_sub;
    boost::shared_ptr<OutlierRemovalSynchronizer> sync;

    ros::Subscriber RGB_img_sub;
    ros::Publisher output_image_pub;
    ros::Publisher remove_outlier_image_pub;

    void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    void RGB_img_Callback(const sensor_msgs::ImageConstPtr&);
    void predict_img_Callback(const sensor_msgs::ImageConstPtr&);
    void RemoveOutlier(Mat &, Mat &, int , int , int , int );
    // void RemoveOutlier(Mat, Mat, Mat);

    void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr& RGBimg_ptr, const sensor_msgs::ImageConstPtr& maskimg_ptr, const LKOpticalFlow_msgs::Points::ConstPtr& points_ptr)
    // void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr& RGBimg_ptr, const sensor_msgs::ImageConstPtr& maskimg_ptr)
    {
        rs_img = cv_bridge::toCvCopy(RGBimg_ptr, "bgr8")->image;
        // mask_img = cv_bridge::toCvCopy(maskimg_ptr, "8UC1")->image;
        mask_img = cv_bridge::toCvCopy(maskimg_ptr, "bgr8")->image;
        
        int nowtime_point_x = (int)points_ptr->point.data.at(0);
        int nowtime_point_y = (int)points_ptr->point.data.at(1);
        int nexttime_point_x = (int)points_ptr->point.data.at(2);
        int nexttime_point_y = (int)points_ptr->point.data.at(3);

        // printf("trackpoint_1 : %f, trackpoint_2 : %f\n",trackpoint_1,trackpoint_2);

        element = getStructuringElement(MORPH_RECT, Size(20, 20));
        dilate(mask_img, mask_img, element);
        
        // RemoveOutlier(rs_img, mask_img, motion_img);
        RemoveOutlier(rs_img, mask_img, nowtime_point_x, nowtime_point_y, nexttime_point_x, nexttime_point_y);
    }
};

ORB_extractor::ORB_extractor(ros::NodeHandle nh){
    img_sub.subscribe(nh, "/camera/color/image_raw", 1);
    mask_sub.subscribe(nh, "/yolact_ros/color/masks_visualization", 1);  // BGR Blue(109,67,14) Red(24,30,109) Green(33,87,62)
    // mask_sub.subscribe(nh, "/yolact_ros/masks/visualization", 1);
    // mask_sub.subscribe(nh, "/ESPNet_v2/mask/image", 1);
    // motion_sub.subscribe(nh, "/moving_check/camera/InstanceSeg_opticalflow_image", 1);
    tracking_sub.subscribe(nh, "/lk/tracked_point_data", 1);
    // sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(300), img_sub, mask_sub, motion_sub));
    // sync->registerCallback(boost::bind(&ORB_extractor::OutlierRemoval_Callback, this, _1, _2, _3));
    sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(220), img_sub, mask_sub, tracking_sub));
    sync->registerCallback(boost::bind(&ORB_extractor::OutlierRemoval_Callback, this, _1, _2, _3));

    // RGB_img_sub = nh.subscribe("/camera/color/image_raw", 1, &ORB_extractor::RGB_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/predict_img", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::mask_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/mask_color_img", 1, &ORB_extractor::predict_img_Callback,this);
    // predict_img_sub = nh.subscribe("moving_check/camera/motion_image", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::OutlierRemoval_Callback,this);
    // keypoints_1_pub = nh.advertise<geometry_msgs::Point32>("/ORBextractor/keypoints", 1);
    output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/color/image", 1);
    remove_outlier_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/color/removeoutlier_image", 1);

    // tracking_nh = nh;
}

ORB_extractor::~ORB_extractor(){
}

// void ORB_extractor::RemoveOutlier(Mat RGB_frame, Mat mask_frame, Mat motion_frame){
void ORB_extractor::RemoveOutlier(Mat &RGB_frame, Mat &mask_frame, int initial_x, int initial_y, int point_x, int point_y){
    // boost::shared_ptr<std_msgs::Float64MultiArray const> trackingpoint_ptr;
    // trackingpoint_ptr = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/lk/tracked_point_data",tracking_nh);
    // cout << "point : " << trackingpoint_ptr->data.at(0) << ", " << trackingpoint_ptr->data.at(1) << ", "<< trackingpoint_ptr->data.at(2) << ", "<< trackingpoint_ptr->data.at(3) << endl;

    assert(RGB_frame.data != nullptr);
    mImGray = RGB_frame;
    cvtColor(mImGray, mImGray, CV_RGB2GRAY);

    cols = mask_frame.cols;
    rows = mask_frame.rows;
    channels = mask_frame.channels();
    // cout << "cols : " << cols << " rows : " << rows << " channels : " << channels << endl;

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    descriptor->compute(mImGray, keypoints_1, descriptors_1);
    
    drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg1).toImageMsg();
    output_image_pub.publish(output_image); 
        
    optical_color_b = mask_frame.at<Vec3b>(initial_y, initial_x)[0];
    optical_color_g = mask_frame.at<Vec3b>(initial_y, initial_x)[1];
    optical_color_r = mask_frame.at<Vec3b>(initial_y, initial_x)[2];

    for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); ++it){
        check = true;
        for(r = 0; r < rows; r++){
            if (check == false){
                break;
            }else{
                for(c = 0; c < cols * channels; c++){
                    if (check == false){
                        break;
                    }else{
                        color_b = mask_frame.at<Vec3b>(r,c)[0];
                        color_g = mask_frame.at<Vec3b>(r,c)[1];
                        color_r = mask_frame.at<Vec3b>(r,c)[2];

                        if ((color_b == optical_color_b) && (color_g == optical_color_g) && (color_r == optical_color_r)){
                            index_x = c;
                            index_y = r;
                            // cout << "match color" << endl;
                            if ((index_x == (int)((*it).pt.x)) && index_y == (int)((*it).pt.y)){
                                // cout << "erase : [" << index_x << " " << index_y << "]" << endl;
                                --(it = keypoints_1.erase(it));
                                check = false;
                            }
                        }
                    }
                }
            }
        }
    }
    // }

    // for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); ++it){
    //     // cnt++;
    //     // cout << cnt << "." << &(*it) <<endl;
    //     // cout << cnt << ". Keypoints : " << (*it).pt << endl;
    //     check = true;
    //     for (r = 0; r <rows; r++){
    //         if (check == false){
    //             break;
    //         }else{
    //             // data = mask_frame.ptr<uchar>(r);
    //             // for(c = 0; c < cols; c++){
    //             for(c = 0; c < cols * mask_frame.channels(); c++){
    //                 if (check == false){
    //                     break;
    //                 }else{
    //                     // if (((int)point_x == c) && ((int)point_y == r)){
    //                         // cout << "point_x : " << point_x << " point_y : " << point_y << endl;
    //                         int color_b = mask_frame.at<Vec3b>(r,c)[0];
    //                         int color_g = mask_frame.at<Vec3b>(r,c)[1];
    //                         int color_r = mask_frame.at<Vec3b>(r,c)[2];

    //                         // index_x = c;
    //                         // index_y = r;
    //                         // cout << "row : " << index_y << " col : " << index_x << " color_b : " << color_b << endl;
    //                         // cout << "color_b : " << color_b << " color_g : " << color_g << " color_r : " << color_r << endl;
    //                         // cout << endl;
                            
    //                         // if((color_b != 0) && (color_g != 0) && (color_r != 0)){
    //                         if((color_b != 0) && (color_g != 0) && (color_r != 0)){
    //                             cnt++;
    //                             index_x = c;
    //                             index_y = r;
    //                             // cout << cnt << ". Make sure segmented pixel." << endl;
    //                             if ((initial_x == index_x) && (initial_y == index_y)){
    //                                 // index_x = c;
    //                                 // index_y = r;
    //                                 // cout << "color_b : " << color_b << " color_g : " << color_g << " color_r : " << color_r << endl;
    //                                 // cout << "index_x : " << index_x << " index_y : " << index_y << endl;
    //                                 // cout << cnt << ". Match optical point with segmented pixel." << endl;
    //                                 // TODO modify to delet the ORB points /////////////////////////////////////////////////////////////////////////////////////////
    //                                 if ((index_x == (int)((*it).pt.x)) && index_y == (int)((*it).pt.y)){
    //                                     cout << cnt << ". Erase ORB feature." << endl;
    //                                     // cout << "index_x : " << index_x << " index_y : " << index_y << endl;
    //                                     // cnt++;
    //                                     // cout << cnt << "." << &(*it) <<endl;
    //                                     // cout << cnt << ". Pixel Erase [x,y] : [" << index_x << "," << index_y << "]" << endl;
    //                                     // cout << cnt << ". Keypoints Erase [x,y] : " << (*it).pt << endl;
    //                                     // cout << endl;
    //                                     --(it = keypoints_1.erase(it));
    //                                     check = false;
    //                                 } 
    //                             }
    //                         }
    //                 }
    //             }
    //         }
    //     }   
    // }

    // for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); ++it){
    //     // cnt++;
    //     // cout << cnt << "." << &(*it) <<endl;
    //     // cout << cnt << ". Keypoints : " << (*it).pt << endl;
    //     check = true;
    //     for (r = 0; r <rows; r++){
    //         if (check == false){
    //             break;
    //         }else{
    //             data = mask_frame.ptr<uchar>(r);
    //             for(c = 0; c < cols * mask_frame.channels(); c++){
    //                 if (check == false){
    //                     break;
    //                 }else{
    //                     index = 640 * r + c;
    //                     mask_label = int(data[c]);
    //                     // if(mask_label == 1){
    //                     if(mask_label == 255){
    //                         index_x = c;
    //                         index_y = r;
    //                         if ((index_x == (int)((*it).pt.x)) && index_y == (int)((*it).pt.y)){
    //                             // cnt++;
    //                             // cout << cnt << "." << &(*it) <<endl;
    //                             // cout << cnt << ". Pixel Erase [x,y] : [" << index_x << "," << index_y << "]" << endl;
    //                             // cout << cnt << ". Keypoints Erase [x,y] : " << (*it).pt << endl;
    //                             // cout << endl;
    //                             --(it = keypoints_1.erase(it));
    //                             check = false;
    //                         }    
    //                     }
    //                 }
    //             }
    //         }
    //     }   
    // }

    drawKeypoints(RGB_frame, keypoints_1, outimg2, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
    remove_outlier_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg2).toImageMsg();
    remove_outlier_image_pub.publish(remove_outlier_image);

    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ORBextractor");
    ros::NodeHandle nh;
    ORB_extractor ORB_extractor(nh);
    ros::spin();
}