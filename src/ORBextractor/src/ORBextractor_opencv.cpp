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
// using namespace message_filters;
// using namespace sensor_msgs;
using namespace cv;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
typedef message_filters::Synchronizer<OutlierRemovalSyncPolicy> OutlierRemovalSynchronizer;

class ORB_extractor{
private:
    // ORBextractor* mpIniORBextractor;
    ros::NodeHandle mask_nh;
    ros::NodeHandle nh_, pnh_;
    unsigned int i_;
    int i , j;
    int index = 0,orb_index = 0;
    int index_x = 0,index_orb_x = 0,index_y = 0,index_orb_y = 0;
    int keypoints_x,keypoints_y;
    int mask_label;
    int channels,orb_channels;
    float tst = 0;
    size_t rows, cols,orb_rows, orb_cols;
    uchar* data;
    Mat result;
    Mat mask_frame;
    Mat mask_ptr_frame;
    Mat RGB_frame,predict_frame;
    Mat mImGray,predict_mImGray;
    Mat outimg1,predict_outimg,outimg2;
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

    void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    void RGB_img_Callback(const sensor_msgs::ImageConstPtr&);
    void predict_img_Callback(const sensor_msgs::ImageConstPtr&);
    void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);
    void RemoveOutlier(Mat &, Mat &, vector<KeyPoint> &);

    ros::Subscriber RGB_img_sub;
    ros::Subscriber mask_img_sub;
    ros::Subscriber predict_img_sub;
    // ros::Subscriber mask_img_sub;
    ros::Publisher keypoints_1_pub;
    ros::Publisher output_image_pub;
    ros::Publisher predict_output_image_pub;
    ros::Publisher remove_outlier_image_pub;
};

ORB_extractor::ORB_extractor(ros::NodeHandle nh){
    // message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/color/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/ESPNet_v2/mask_img", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img_sub, mask_sub, 10);
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, mask_sub);
    // sync.registerCallback(boost::bind(&ORB_extractor::OutlierRemoval_Callback, this, _1, _2));

    img_sub.subscribe(nh, "/camera/color/image_raw", 1);
    mask_sub.subscribe(nh, "/ESPNet_v2/mask_img", 1);
    sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(10), img_sub, mask_sub));
    sync->registerCallback(boost::bind(&ORB_extractor::OutlierRemoval_Callback, this, _1, _2));

    // RGB_img_sub = nh.subscribe("/camera/color/image_raw", 1, &ORB_extractor::RGB_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/predict_img", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::mask_img_Callback,this);
    // predict_img_sub = nh.subscribe("/ESPNet_v2/mask_color_img", 1, &ORB_extractor::predict_img_Callback,this);
    // predict_img_sub = nh.subscribe("moving_check/camera/motion_image", 1, &ORB_extractor::predict_img_Callback,this);
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &ORB_extractor::OutlierRemoval_Callback,this);
    // keypoints_1_pub = nh.advertise<geometry_msgs::Point32>("/ORBextractor/keypoints", 1);
    // output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/image", 1);
    // predict_output_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/predict_image", 1);
    // remove_outlier_image_pub = nh.advertise<sensor_msgs::Image>("/ORBextractor/removeoutlier_image", 1);

    mask_nh = nh;
}

ORB_extractor::~ORB_extractor(){
}

void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr& RGBimg_msg, const sensor_msgs::ImageConstPtr& maskimg_msg){
    cout << "rgb_msg timestamp : " << RGBimg_msg->header.stamp << endl;
    cout << "mask_msg timestamp : " << maskimg_msg->header.stamp << endl;
    cout << endl;
}

void ORB_extractor::mask_img_Callback(const sensor_msgs::ImageConstPtr& maskimg_msg){
    cout << "mask_msg timestamp : " << maskimg_msg->header.stamp << endl;
}

void ORB_extractor::RGB_img_Callback(const sensor_msgs::ImageConstPtr& RGBimg_msg){
    RGB_frame = cv_bridge::toCvCopy(RGBimg_msg, "bgr8")->image;
    assert(RGB_frame.data != nullptr);
    mImGray = RGB_frame;
    cvtColor(mImGray, mImGray, CV_RGB2GRAY);

    cout << "rgb_msg timestamp : " << RGBimg_msg->header.stamp << endl;

    // vector<KeyPoint> keypoints_1, keypoints_2;
    // Mat descriptors_1, descriptors_2;
    // keypoints_1.resize(100);

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    descriptor->compute(mImGray, keypoints_1, descriptors_1);

    drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg1).toImageMsg();
    output_image_pub.publish(output_image);
}

void ORB_extractor::predict_img_Callback(const sensor_msgs::ImageConstPtr& predict_img_msg){
    predict_frame = cv_bridge::toCvCopy(predict_img_msg, "bgr8")->image;
    assert(predict_frame.data != nullptr);
    predict_mImGray = predict_frame;
    cvtColor(predict_mImGray, predict_mImGray, CV_RGB2GRAY);

    boost::shared_ptr<sensor_msgs::Image const> mask_ptr;
    mask_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/ESPNet_v2/mask_img",mask_nh);
    mask_ptr_frame = cv_bridge::toCvCopy(mask_ptr, "8UC1")->image;

    cout << predict_img_msg << endl;
    cout << endl;

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(predict_mImGray, predict_keypoints_1);
    descriptor->compute(predict_mImGray, predict_keypoints_1, predict_descriptors_1);
    
    // if ((predict_img_msg->header.stamp) == (mask_ptr->header.stamp)){
    //     try{
    //         RemoveOutlier(mask_ptr_frame, result, predict_keypoints_1);
    //     }
    //     catch(const std::exception& e){
    //         std::cerr << e.what() << '\n';
    //     }
    // }
    
    

    drawKeypoints(predict_frame, predict_keypoints_1, predict_outimg, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    predict_output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", predict_outimg).toImageMsg();
    predict_output_image_pub.publish(predict_output_image);
}

void ORB_extractor::RemoveOutlier(Mat &frame, Mat &outimg2, vector<KeyPoint> &keypoints){
    cols = frame.cols;
    rows = frame.rows;
    channels = frame.channels();
    // cout << "Size:" << mask_frame.size() << " " << "Column: " << cols << " " << "Row: " << rows << " " << "Channels: " << channels << endl;
    for(vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++){
        for (i = 0; i <rows; i++){
            data = frame.ptr<uchar>(i);
            for(j = 0; j < cols * frame.channels(); j++){
                index = 640 * i + j;
                mask_label = int(data[j]);
                if(mask_label == 1){
                    index_x = i;
                    index_y = j;
                    if ((index_x = (*it).pt.x) && (index_y = (*it).pt.y)){
                        cout << "okok" << endl;
                        // cout << "Pixel [x,y] : [" << index_x << "," << index_y << "]" << endl;
                        // cout << "Keypoints [x,y] : [" << (*it).pt.x << "," << (*it).pt.y << "]" << endl;
                        // cout << endl;
                        keypoints.erase(it);
                        it = keypoints.begin();
                    }                    
                }
            }
        }
    }

    drawKeypoints(RGB_frame, keypoints, outimg2, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
    remove_outlier_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outimg2).toImageMsg();
    remove_outlier_image_pub.publish(remove_outlier_image);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ORBextractor");
    ros::NodeHandle nh, pnh("~");
    ORB_extractor ORB_extractor(nh);
    ros::spin();
}