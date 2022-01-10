#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point32.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace ros;
using namespace std;
using namespace cv;
using namespace message_filters;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;

class Outlier_Removal{
private:
    ros::NodeHandle orb_nh;
    ros::NodeHandle keypoints_nh;
    
    int i ,j;
    int index = 0,orb_index = 0;
    int index_i = 0,index_orb_i = 0,index_y = 0,index_orb_y = 0;
    int keypoints_x,keypoints_y;
    int mask_label;
    size_t rows, cols,orb_rows, orb_cols;
    int channels,orb_channels;
    uchar* data;

    Mat orb_ptr_frame;
    Mat mask_frame;
    Vec3b PixelValue;
    uint8_t* pixelPtr;
    int cn;
    Scalar_<uint8_t> bgrPixel;

    sensor_msgs::ImagePtr output_tstimage;

public:
    Outlier_Removal(ros::NodeHandle);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
    // message_filters::Subscriber<sensor_msgs::Image>* ORB_sub ;  
    // message_filters::Subscriber<sensor_msgs::Image>* mask_sub ;  
    // message_filters::Synchronizer<OutlierRemovalSyncPolicy>* sync;
    void OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr&);
    ros::Subscriber mask_img_sub;
    ros::Publisher output_tstimg_pub;
    // image_transport::Subscriber mask_img_sub;
    // image_transport::Publisher output_tstimg_pub;
};

Outlier_Removal::Outlier_Removal(ros::NodeHandle nh){
    // ORB_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/ORBextractor/image", 1);
    // mask_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/ESPNet_v2/mask_img", 1);
    // sync = new  message_filters::Synchronizer<OutlierRemovalSyncPolicy>(OutlierRemovalSyncPolicy(10), *ORB_sub, *mask_sub);
    // sync->registerCallback(boost::bind(&Outlier_Removal::OutlierRemoval_Callback,this, _1, _2));
    
    // image_transport::ImageTransport it(nh);

    mask_img_sub = nh.subscribe("/ESPNet_v2/mask_img", 1, &Outlier_Removal::OutlierRemoval_Callback,this);
    output_tstimg_pub = nh.advertise<sensor_msgs::Image>("moving_check/tst_image", 1);

    orb_nh = nh;
    keypoints_nh = nh;
}

void Outlier_Removal::OutlierRemoval_Callback(const sensor_msgs::ImageConstPtr& mask_msg){
    mask_frame = cv_bridge::toCvCopy(mask_msg, "8UC1")->image;

    boost::shared_ptr<geometry_msgs::Point32 const> keypoints_ptr;
    keypoints_ptr = ros::topic::waitForMessage<geometry_msgs::Point32>("/ORBextractor/keypoints",keypoints_nh);

    boost::shared_ptr<sensor_msgs::Image const> orb_ptr;
    orb_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/ORBextractor/image",orb_nh);
    // orb_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/ESPNet_v2/mask_color_img",orb_nh);
    orb_ptr_frame = cv_bridge::toCvCopy(orb_ptr, "bgr8")->image;

    // cout << "Keypoints [x,y,z] : [" << (int)keypoints_ptr->x << "," << (int)keypoints_ptr->y << "," << (int)keypoints_ptr->z << "]"<< endl;

    // if ((orb_ptr->header.stamp) == (mask_msg->header.stamp)){
    //     cout << i << ". " << "ORB_frame TimeStamp : " << orb_ptr->header.stamp << endl;
    //     cout << i << ". " << "mask_frame TimeStamp : " << mask_msg->header.stamp << endl;
    //     cout << endl;
    //     i++;
    // }else{
    //     cout << "No Synchronize" << endl;
    // }
    
    // cv::Mat dstImage(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));
    // dstImage = frame.clone();
    // // cout << frame << endl;
    // // cout << "**********************************************************" << endl;
    // // cout << "Size:" << frame.size() << " " << "Column: " << frame.cols << " " << "Row: " << frame.rows << endl;

    keypoints_x = (int)keypoints_ptr->x;
    keypoints_y = (int)keypoints_ptr->y;

    cols = mask_frame.cols;
    rows = mask_frame.rows;
    channels = mask_frame.channels();
    // cout << "Size:" << mask_frame.size() << " " << "Column: " << cols << " " << "Row: " << rows << " " << "Channels: " << channels << endl;
    for (i = 0; i <rows; i++){
        data = mask_frame.ptr<uchar>(i);
        for(j = 0; j < cols * mask_frame.channels(); j++){
            index = 640 * i + j;
            mask_label = int(data[j]);
            index_i = i;
            index_y = j;
            if(mask_label == 1){
                // cout << "Index: " << index << " " << "Label: " << mask_label << " " << "Mask Column: " << i  << " " << "Mask Row: " << j << endl;
                if((index_i = keypoints_x) && (index_y = keypoints_y)){
                    cout << "Keypoints [x,y] : [" << keypoints_x << "," << keypoints_y << "]" << endl;
                }
            }
        }
    }

    orb_cols = orb_ptr_frame.cols;
    orb_rows = orb_ptr_frame.rows;
    orb_channels = orb_ptr_frame.channels();
    // cout << orb_ptr_frame << endl;
    // cout << endl;
    // cout << "Size:" << orb_ptr_frame.size() << " " << "Column: " << orb_cols << " " << "Row: " << orb_rows << " " << "Channels: " << orb_channels << endl;
    // for (i = 0; i <orb_rows; i++){
    //     for(j = 0; j < orb_cols; j++){
    //         orb_index = 640 * i + j;
    //         // index_orb_i = i;
    //         // index_orb_y = j;
    //         if((i=index_i) && (j=index_y)){
    //             cout << "orb_Index: " << orb_index << " " << "orb_Column: " << i  << " " << "orb_Row: " << j << endl;
    //         }
    //     }
    // }


    // cout << "Mask_img: " << endl;
    // for (i = 0; i <rows; i++){
    //     uchar* data = frame.ptr<uchar>(i);
    //     for(j = 0; j < cols * frame.channels(); j++){
    //         int index = 640 * i + j;
    //         int mask_label = int(data[j]);
    //         if(mask_label == 1){
    //             cout << "000" << endl;
    //             dstImage.at<uchar>(i, j) = 255;
    //             cout << "111" << endl;
    //             // cout << "Index: " << index << endl;
    //         }
    //         // cout << int( data[j] );
    //     }
    //     // cout << " " << endl;
    // }
    // output_tstimage = cv_bridge::CvImage(std_msgs::Header(), "8UC1", dstImage).toImageMsg();
	// output_tstimg_pub.publish(output_tstimage);
        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Outlier_Removal");
    ros::NodeHandle nh;
    Outlier_Removal Outlier_Removal(nh);
    ros::spin();
}