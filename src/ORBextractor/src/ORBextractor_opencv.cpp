#include <iostream>
#include <cstdio>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point32.h>


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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
typedef message_filters::Synchronizer<OutlierRemovalSyncPolicy> OutlierRemovalSynchronizer;

class ORB_extractor{
private:
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
    size_t rows, cols,orb_rows, orb_cols;
    uchar* data;
    Mat result;
    Mat mask_img;
    Mat rs_img,predict_frame;
    Mat mImGray,predict_mImGray;
    Mat outimg1,predict_outimg,outimg2;
    Mat element;
    vector<KeyPoint> keypoints_1, predict_keypoints_1, keypoints_2;
    Point2f outlier_points;
    Mat descriptors_1, predict_descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

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
    boost::shared_ptr<OutlierRemovalSynchronizer> sync;

    ros::Subscriber RGB_img_sub;
    ros::Subscriber mask_img_sub;
    ros::Subscriber predict_img_sub;
    ros::Publisher keypoints_1_pub;
    ros::Publisher output_image_pub;
    ros::Publisher predict_output_image_pub;
    ros::Publisher remove_outlier_image_pub;

    void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    void RGB_img_Callback(const sensor_msgs::ImageConstPtr&);
    void predict_img_Callback(const sensor_msgs::ImageConstPtr&);
    // void RemoveOutlier(Mat &, Mat &, vector<KeyPoint> &);
    void RemoveOutlier(Mat, Mat);

    void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr& RGBimg_ptr, const sensor_msgs::ImageConstPtr& maskimg_ptr)
    {
        rs_img = cv_bridge::toCvCopy(RGBimg_ptr, "bgr8")->image;
        mask_img = cv_bridge::toCvCopy(maskimg_ptr, "8UC1")->image;
        element = getStructuringElement(MORPH_RECT, Size(8, 8));
        dilate(mask_img, mask_img, element);
        RemoveOutlier(rs_img, mask_img);

        // cout << "rgb_msg timestamp : " << RGBimg_ptr->header.stamp << endl;
        // cout << "mask_msg timestamp : " << maskimg_ptr->header.stamp << endl;
        // cout << endl;
    }
};

ORB_extractor::ORB_extractor(ros::NodeHandle nh){
    img_sub.subscribe(nh, "/camera/color/image_raw", 1);
    mask_sub.subscribe(nh, "/ESPNet_v2/mask_img", 1);
    sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(180), img_sub, mask_sub));
    sync->registerCallback(boost::bind(&ORB_extractor::OutlierRemoval_Callback, this, _1, _2));

    // RGB_img_sub = nh.subscribe("/camera/color/image_raw", 1, &ORB_extractor::RGB_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/predict_img", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::mask_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/mask_color_img", 1, &ORB_extractor::predict_img_Callback,this);
    // predict_img_sub = nh.subscribe("moving_check/camera/motion_image", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::OutlierRemoval_Callback,this);
    // keypoints_1_pub = nh.advertise<geometry_msgs::Point32>("/ORBextractor/keypoints", 1);
    output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/image", 1);
    predict_output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/predict_image", 1);
    remove_outlier_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/removeoutlier_image", 1);
}

ORB_extractor::~ORB_extractor(){
}

void ORB_extractor::RemoveOutlier(Mat RGB_frame, Mat mask_frame){
    // cout << "ok" << endl;
    assert(RGB_frame.data != nullptr);
    mImGray = RGB_frame;
    cvtColor(mImGray, mImGray, CV_RGB2GRAY);

    cols = mask_frame.cols;
    rows = mask_frame.rows;
    channels = mask_frame.channels();

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    descriptor->compute(mImGray, keypoints_1, descriptors_1);

    // for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); it++){
    //     if ((*it).pt.x > 427){
    //             // cout << "KeyPoints : " << (*it).pt << endl;
    //             keypoints_1.erase(it);
    //             it = keypoints_1.begin();
    //     }
    //     if ((*it).pt.x < 213){
    //             // cout << "KeyPoints : " << (*it).pt << endl;
    //             keypoints_1.erase(it);
    //             it = keypoints_1.begin();
    //     }
    //     if ((*it).pt.y < 160){
    //             // cout << "KeyPoints : " << (*it).pt << endl;
    //             keypoints_1.erase(it);
    //             it = keypoints_1.begin();
    //     }
    //     if ((*it).pt.y > 320){
    //             // cout << "KeyPoints : " << (*it).pt << endl;
    //             keypoints_1.erase(it);
    //             it = keypoints_1.begin();
    //     }
    //     cout << "KeyPoints : " << (*it).pt << endl;
    // }
    
    // drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    // output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg1).toImageMsg();
    // output_image_pub.publish(output_image);

        // cout << "Keypoints [x,y] : [" << (*it).pt.x << "," << (*it).pt.y << "]" << endl;
    for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); ++it){
        cnt++;
        // cout << cnt << "." << &(*it) <<endl;
        // cout << cnt << ". Keypoints : " << (*it).pt << endl;
        check = true;
        for (r = 0; r <rows; r++){
            if (check == false){
                break;
            }else{
                data = mask_frame.ptr<uchar>(r);
                for(c = 0; c < cols * mask_frame.channels(); c++){
                    if (check == false){
                        break;
                    }else{
                        index = 640 * r + c;
                        mask_label = int(data[c]);
                        if(mask_label == 1){
                            index_x = c;
                            index_y = r;
                            // cout << cnt << ". Pixel [x,y] : [" << index_x << "," << index_y << "]" << endl;
                            if ((index_x == (int)((*it).pt.x)) && index_y == (int)((*it).pt.y)){
                                // cnt++;
                                // cout << cnt << "." << &(*it) <<endl;
                                // cout << cnt << ". Pixel Erase [x,y] : [" << index_x << "," << index_y << "]" << endl;
                                // cout << cnt << ". Keypoints Erase [x,y] : " << (*it).pt << endl;
                                // cout << endl;
                                --(it = keypoints_1.erase(it));
                                // it = keypoints_1.erase(it);
                                check = false;
                            }    
                        }
                    }

                }
            }
        }   
    }

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