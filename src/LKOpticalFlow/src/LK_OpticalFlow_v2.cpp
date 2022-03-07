#include <ros/ros.h>
// #include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>
#include <math.h>
// #include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "LKOpticalFlow_msgs/Points.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/common.h>

// #include <sensor_msgs/image_encodings.h>
using namespace ros;
using namespace std;
using namespace cv;
using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

typedef pcl::PointXYZRGB originpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> OriginPointCloudXYZRGBtoROSMsg;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginPointCloudXYZRGBtoROSMsgPtr;

typedef pcl::PointXYZRGB targetpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> TargetPointCloudXYZRGBtoROSMsg;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr TargetPointCloudXYZRGBtoROSMsgPtr;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> OutlierRemovalSyncPolicy;
typedef message_filters::Synchronizer<OutlierRemovalSyncPolicy> OutlierRemovalSynchronizer;

class LK_OpticalFlow
{
private:
    ros::NodeHandle my_nh;
    ros::NodeHandle rgb_nh;

    cv::Mat frame;
    cv::Mat raw_frame;
    cv::Mat mask_frame;
    cv::Mat rgb_ptr_frame;
    Mat result;
    Mat gray;   // 當前圖片
    Mat gray_prev;  // 預測圖片

    cv_bridge::CvImagePtr cv_ptr; 
    string window_name = "Lucas–Kanade Optical Flow Tracking";
    vector<Point2f> points[2];  // point0為特徵點的原来位置，point1為特徵點的新位置
    vector<Point2f> initial;    // 初始化跟蹤點的位置
    vector<Point2f> features;   // 檢測的特徵
    int maxCount = 500; // 檢測的最大角點數目
    double qLevel = 0.01;   // 特徵檢測的等級（一般於0.01-0.1之間）
    double minDist = 10.0;  // 兩特徵點之間的最小距離，小於此距離的點要被忽略
    vector<uchar> status;   // 跟蹤特徵的狀態，特徵的流發現為1，否則為0
    vector<float> err;

    // clock_t t1, t2, delta_time;
    int i, j, r, c;
    int rows, cols, channels;
    int rgb_rows, rgb_cols, rgb_channels;
    int count = 0;
    // int time_header;
    double deltaDist;

	float x_op_original;
	float y_op_original;
	float z_op_original;
	float x_op_target;
	float y_op_target;
	float z_op_target;
	// float distance;
	float EuclideanDis;
	float EuDis;
	float D4Dis;
	float CityBlockDis;
	float topic_t;
	double vel;
    double begin_time;
    double end_time;
    double delta_time;

    uchar* mask_data;
    int mask_index;
    int mask_label;
    int tmp_index;
    int rgb_index;

    geometry_msgs::Vector3 feature_points_msg;
    sensor_msgs::PointCloud2 originPointCloudtoROSMsg;
    sensor_msgs::PointCloud2 targetPointCloudtoROSMsg;
	sensor_msgs::ImagePtr output_image;
    sensor_msgs::ImagePtr output_tstimage;
	std_msgs::Float64MultiArray origin_point;
	std_msgs::Float64MultiArray target_point;
	std_msgs::Float64MultiArray point_data;
	std_msgs::Float64MultiArray distance_data;
	std_msgs::Float64MultiArray vel_data;
    std_msgs::Header header;
    LKOpticalFlow_msgs::Points tracked_point_data;
    
    
    PointCloudXYZRGBPtr cloud_ptr;
    originpoint origin_pointcloud;
    OriginPointCloudXYZRGBtoROSMsgPtr origincloudptr_to_ROSMsg;
    targetpoint target_pointcloud;
    TargetPointCloudXYZRGBtoROSMsgPtr targetcloudptr_to_ROSMsg;

public:
    LK_OpticalFlow(ros::NodeHandle);
    ~LK_OpticalFlow();
    message_filters::Subscriber<sensor_msgs::Image> raw_sub;
    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    message_filters::Subscriber<sensor_msgs::Image> mask_sub;
    boost::shared_ptr<OutlierRemovalSynchronizer> sync;

    // void mask_img_Callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);
    void mask_img_Callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);
    // void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    // void image_raw_Callback(const sensor_msgs::ImageConstPtr&);
    void Tracking(Mat &, Mat &);
    bool AddNewPoints();
    bool AcceptTrackedPoint(int);
	float EuclideanDistance(float, float, float, float);
	float D4Distance(float, float, float, float);
	// float ComputeDistance(float, float, float, float, float, float);
	// float ComputeVelocity(float, clock_t);
	ros::Subscriber image_raw_sub;
    ros::Publisher feature_points_pub;
    ros::Publisher origin_pointcloud_pub;
    ros::Publisher target_pointcloud_pub;
	ros::Publisher origin_point_pub;	
	ros::Publisher target_point_pub;
	ros::Publisher point_data_pub;
	ros::Publisher distance_data_pub;
	ros::Publisher vel_data_pub;
	ros::Publisher tracked_point_data_pub;
    ros::Publisher output_image_pub;
    ros::Publisher output_motion_pub;
    ros::Subscriber mask_img_sub;

    // std_msgs::Header header;

    // message_filters::Subscriber<sensor_msgs::Image> predictImage_sub;
    // message_filters::Subscriber<sensor_msgs::Image> maskImage_sub;

    // image_transport::Subscriber mask_img_sub;
	// image_transport::Publisher output_image_pub;
    // ros::Publisher test_image_pub;
};

LK_OpticalFlow::LK_OpticalFlow(ros::NodeHandle nh)
{
    image_transport::ImageTransport it(nh);

    // img_sub.subscribe(nh, "/ESPNet_v2/color/predict_image", 1);
    // mask_sub.subscribe(nh, "/ESPNet_v2/mask/image", 1);

    raw_sub.subscribe(nh, "/camera/color/image_raw", 1);
    mask_sub.subscribe(nh, "/yolact_ros/masks/visualization", 1);
    // mask_sub.subscribe(nh, "/yolact_ros/color/visualization", 1);
    // img_sub.subscribe(nh, "/yolact_ros/color/visualization", 1);
    
    sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(220), raw_sub, mask_sub));
    sync->registerCallback(boost::bind(&LK_OpticalFlow::mask_img_Callback, this, _1, _2));
    // sync.reset(new OutlierRemovalSynchronizer(OutlierRemovalSyncPolicy(220), mask_sub, img_sub));
    // sync->registerCallback(boost::bind(&LK_OpticalFlow::mask_img_Callback, this, _1, _2));
    
    // mask_img_sub = nh.subscribe("/ESPNet_v2/mask/image", 1, &LK_OpticalFlow::mask_img_Callback,this);

    // image_raw_sub = nh.subscribe("/camera/color/image_raw", 1, &LK_OpticalFlow::image_raw_Callback,this);
    // image_raw_sub = nh.subscribe("/ESPNet_v2/predict_img", 1, &LK_OpticalFlow::image_raw_Callback,this);
    // message_filters::Subscriber<sensor_msgs::Image> predictImage_sub(nh, "/ESPNet_v2/predict_img", 1);
    // message_filters::Subscriber<sensor_msgs::Image> maskImage_sub(nh, "/ESPNet_v2/mask_img", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(predictImage_sub, maskImage_sub, 10);
    // sync.registerCallback(boost::bind(&LK_OpticalFlow::image_raw_Callback, this, _1, _2));
    // image_raw_sub = nh.subscribe("/camera/color/image_raw", 1, &LK_OpticalFlow::image_raw_Callback,this);
    // feature_points_pub = nh.advertise<geometry_msgs::Vector3>("lk/feature_points", 1);
    // feature_points_pub = nh.advertise<geometry_msgs::Point32>("feature_points", 10);
    // origin_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lk/origin_pointcloud", 1);
    // target_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lk/target_pointcloud", 1);
	// origin_point_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/origin_point", 1);
	// target_point_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/target_point", 1);
	// point_data_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/point_data", 1);
	// distance_data_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/distance_data", 1);
	// vel_data_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/vel_data", 1);
	// tracked_point_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/lk/tracked_point_data", 1);
    tracked_point_data_pub = nh.advertise<LKOpticalFlow_msgs::Points>("/lk/tracked_point_data", 1);

	// output_image_pub = it.advertise("lk/camera/LK_OpticalFlow_image", 1);
    // output_image_pub = it.advertise("moving_check/camera/semantic_opticalflow_image", 1);
    output_image_pub = nh.advertise<sensor_msgs::Image>("/moving_check/camera/InstanceSeg_opticalflow_image", 1);
    output_motion_pub = nh.advertise<sensor_msgs::Image>("/moving_check/camera/motion_image", 1);

    //cv::namedWindow(window_name);
	const int width = 640; // 設定影像尺寸(寬w，高h)
	const int high = 480;

    my_nh = nh;
    rgb_nh = nh;
    cloud_ptr = PointCloudXYZRGBPtr(new PointCloudXYZRGB);
    origincloudptr_to_ROSMsg = OriginPointCloudXYZRGBtoROSMsgPtr(new OriginPointCloudXYZRGBtoROSMsg);
    targetcloudptr_to_ROSMsg = TargetPointCloudXYZRGBtoROSMsgPtr(new TargetPointCloudXYZRGBtoROSMsg);    
}

LK_OpticalFlow::~LK_OpticalFlow()
{
}

void LK_OpticalFlow::mask_img_Callback(const sensor_msgs::ImageConstPtr& raw_msg, const sensor_msgs::ImageConstPtr& mask_msg){
// void LK_OpticalFlow::mask_img_Callback(const sensor_msgs::ImageConstPtr& mask_msg, const sensor_msgs::ImageConstPtr& rgb_ptr){
// void LK_OpticalFlow::mask_img_Callback(const sensor_msgs::ImageConstPtr& mask_msg){
    // cout << "okokokokok" <<endl;
    raw_frame = cv_bridge::toCvCopy(raw_msg, "bgr8")->image;
    mask_frame = cv_bridge::toCvCopy(mask_msg, "8UC1")->image;
    // rgb_ptr_frame = cv_bridge::toCvCopy(rgb_ptr, "bgr8")->image;

    header = raw_msg->header;
    // time_header = raw_msg->header.stamp.sec;
    // cout << "time_stamp : " << time_header << endl;
    // int time_stamp = time_header.header.stamp.secs;

    // if(!mask_frame.empty()){
    //     cout << "mask_frame" << endl;
    // }
    // cv::Mat dstImage(mask_frame.rows, mask_frame.cols, CV_8UC3, cv::Scalar::all(0));
    // dstImage = mask_frame.clone();
    // cout << frame << endl;
    // cout << "**********************************************************" << endl;
    // cout << "Size:" << frame.size() << " " << "Column: " << frame.cols << " " << "Row: " << frame.rows << endl;

    // boost::shared_ptr<sensor_msgs::Image const> rgb_ptr;
    // rgb_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw",rgb_nh);
    // rgb_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/ESPNet_v2/color/predict_image",rgb_nh);
    // rgb_ptr_frame = cv_bridge::toCvCopy(rgb_ptr, "bgr8")->image;
    
    // cv::Mat dstImage(rgb_ptr_frame.rows, rgb_ptr_frame.cols, CV_8UC3, cv::Scalar::all(0));
    cv::Mat dstImage(raw_frame.rows, raw_frame.cols, CV_8UC3, cv::Scalar::all(0));
    // dstImage = rgb_ptr_frame.clone();

    // cout << "dstImage: " << dstImage << endl;

    cols = mask_frame.cols;
    rows = mask_frame.rows;
    channels = mask_frame.channels();

    // cout << "Mask_img: " << endl;
    for (i = 0; i <rows; i++){
        mask_data = mask_frame.ptr<uchar>(i);
        for(j = 0; j < cols * mask_frame.channels(); j++){
            mask_index = 640 * i + j;
            mask_label = int(mask_data[j]);
            // cout << "Label: " << mask_label << "," << "Index: " << mask_index << "," << "Pixel at position (x, y): (" << j << "," << i << ") = " << mask_frame.at<Vec3b>(i,j) << endl;
            if(mask_label == 255){
                // mask_label = 1;
                dstImage.at<Vec3b>(i, j) = raw_frame.at<Vec3b>(i,j);
                // dstImage.at<Vec3b>(i, j) = rgb_ptr_frame.at<Vec3b>(i,j);
                // Vec3b rgb_pixel = rgb_ptr_frame.at<Vec3b>(i,j);
                // Vec3b dstImage_pixel = dstImage.at<Vec3b>(i,j);
                // dstImage_pixel[0] = rgb_pixel[0];
                // dstImage_pixel[1] = rgb_pixel[1];
                // dstImage_pixel[2] = rgb_pixel[2];
                // dstImage.at<uchar>(i, j) = 255;
                // tmp_index = mask_index;
            }
        }
    }
    
    
        // cout << raw_frame.size() << "," << mask_frame.size() << "," << dstImage.size() << endl;

        // cout << "dstImage: " << dstImage << endl;
        // cout << "**********************************************" << endl;
    
    output_tstimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dstImage).toImageMsg();
	output_motion_pub.publish(output_tstimage);

    try{
        // Tracking(rgb_ptr_frame, result);
        // Tracking(dstImage, result);
        Tracking(dstImage, result);
    }catch(cv::Exception& e){
        const char* err_msg = e.what();
        cout << "Exception caught: " << err_msg << endl;
    }
}

void LK_OpticalFlow::Tracking(Mat &frame, Mat &output)
{
    // printf( "\033[1;32;1m ************************ Start Tracking ************************\n \033[0m" );
    // cout << "tracking" << endl;
    cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);
    // cout << "00" << endl;
    // 添加特徵點
    if (AddNewPoints()) 
    {
        //角點檢測
        //image:輸入圖像(gray)
        //corners:輸出角點vector(features)
        //maxCorners:檢測的最大角點數目(maxCount)
        //qualityLevel:特徵檢測的等級,一般於0.01-0.1之間(qLevel)
        //minDistance:兩特徵點之間的最小距離，小於此距離的點要被忽略(minDist)
        goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
        
        // cout << "============features vector============" << endl;
        // cout << "features : " << features << endl;
        // cout << "====================================\n" << endl;

        points[0].insert(points[0].end(), features.begin(), features.end());
        // cout << "============points[0] vector============" << endl;
        // cout << "points[0] : " << points[0] << endl;
        // cout << "====================================\n" << endl;

        initial.insert(initial.end(), features.begin(), features.end());
        // cout << "============initial vector============" << endl;
        // cout << "initial : " << initial << endl;
        // cout << "====================================\n" << endl;
    }

    if (gray_prev.empty())
    {
        // cout << gray_prev <<endl;
        gray.copyTo(gray_prev);
    }
    // LK-光流法運動估計
    //prevImg:計算光流的前一偵圖片(gray_prev）
    //nextImg:下一偵圖片(gray)
    //prevPts:前一偵圖片中的特徵點（角點）,
    //nextPts:輸出特徵點於下一偵圖片的新位置
    //status:若兩偵之間的特徵點有發生變化（有光流法現象）則為1，否則為0
    //err:兩偵之間特徵點位置的誤差
    // cout << "01" << endl;
    calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err); // LK-光流法運動估計
    // cout << "11" << endl;
    // cout << "w: " << gray.cols << ", h: " << gray.rows << endl;
    // 去掉一些不好的特徵點
    int k = 0;
    for (size_t i=0; i<points[1].size(); i++)
    {
        // cout << "points[1][" << i <<"]:" << points[1][i] << endl;
        if (AcceptTrackedPoint(i))
        {
            EuclideanDis = sqrt(pow((points[0][i].x - points[1][i].x),2) + pow((points[0][i].y - points[1][i].y),2));
            // if (EuclideanDis > 100){ //0 20 50 100
            // // printf("%hhu\n",status[i]);
            // // cout << deltaDist <<endl;
            // initial[k] = initial[i];
            // points[1][k++] = points[1][i];
            // }

            initial[k] = initial[i];
            points[1][k++] = points[1][i];
        }
        // cout << "k_points[1][" << k++ << "]:" << points[1][k++] << endl;
        // cout <<endl;
    }
    points[1].resize(k);
    initial.resize(k);
    // 顯示特徵點和運動軌跡
    for (size_t i=0; i<points[1].size(); i++)
    {
        // EuclideanDis = sqrt(pow(((float)initial[i].x - (float)initial[i].y),2) + pow(((float)points[1][i].x - (float)points[1][i].y),2));
        // if (EuclideanDis > 50){
        //     (initial[i].x) = (initial[i].x);
        //     (initial[i].y) = (initial[i].y);
        //     (points[1][i].x) = (points[1][i].x);
        //     (points[1][i].y) = (points[1][i].y);
        // }

        line(output, initial[i], points[1][i], Scalar(255, 0, 0), 1);
        circle(output, initial[i], 4, Scalar(0, 255, 0), -1);
        circle(output, points[1][i], 4, Scalar(0, 0, 255), -1);

		// printf("\033[1;33m Points pixel [ init_x init_y end_x end_y]: [ %lf %lf %lf %lf ] \033[0m \033[1;31m EuclideanDis : %f \033[0m \033[1;34m D4Dis : %f\n \033[0m", (float)initial[i].x, (float)initial[i].y, (float)points[1][i].x, (float)points[1][i].y, EuclideanDis, D4Dis);
        
		EuDis = EuclideanDistance((float)initial[i].x, (float)initial[i].y, (float)points[1][i].x, (float)points[1][i].y);
		CityBlockDis = D4Distance((float)initial[i].x, (float)initial[i].y, (float)points[1][i].x, (float)points[1][i].y);
		// printf("\033[1;33m Points pixel [ init_x init_y end_x end_y]: [ %lf %lf %lf %lf ] \033[0m", (float)initial[i].x, (float)initial[i].y, (float)points[1][i].x, (float)points[1][i].y);
		
        tracked_point_data.header = header;
        // tracked_point_data.header.stamp.sec = timestamp;
        tracked_point_data.point.data.push_back((float)initial[i].x);
		tracked_point_data.point.data.push_back((float)initial[i].y);
		tracked_point_data.point.data.push_back((float)points[1][i].x);
		tracked_point_data.point.data.push_back((float)points[1][i].y);
		// tracked_point_data.data.push_back(EuDis);
		// tracked_point_data.data.push_back(CityBlockDis);
        cout << "Tracked Point : \n" << tracked_point_data << endl;
		tracked_point_data_pub.publish(tracked_point_data);
		tracked_point_data.point.data.clear();

		//cout<< "==================================" << endl;
        //cout << "InitialPoint : " << initial[i] << endl;
        //cout << "TerminalPoint : " << points[1][i] << endl;
        //cout<< "==================================\n" << endl;
        // double opticaldistence = sqrt(pow((points[0][i].x - points[1][i].x),2)+pow((points[0][i].y - points[1][i].y),2));
        //cout<< "**************************" << endl;
        // cout << "opticaldistence : " << opticaldistence << endl;
        
        // ROS_INFO("Origin feature point: x=%f, y=%f", (float)initial[i].x, (float)initial[i].y);
        // ROS_INFO("Target feature point: x=%f, y=%f\n", (float)points[1][i].x, (float)points[1][i].y);
        // feature_points_msg.x = points[1][i].x;
        // feature_points_msg.y = points[1][i].y;
        // feature_points_msg.z = 0;
        // feature_points_pub.publish(feature_points_msg);
//*******************************************************************************************************************
        // int initial_index = 640 * initial[i].y + initial[i].x; //計算Origin point單一pixel-wise於640x480第幾個
        // if(pcl_isfinite(cloud_ptr->points[initial_index].x)) 
        // {   //ROS_INFO("\033[1;32m\nOrigin point[x y z] : [%lf %lf %lf]\033[0m",cloud_ptr->points[initial_index].x,cloud_ptr->points[initial_index].y,cloud_ptr->points[initial_index].z);

        //     origin_pointcloud.r = 0;
        //     origin_pointcloud.g = 255;
        //     origin_pointcloud.b = 0;
        //     origin_pointcloud.x = cloud_ptr->points[initial_index].x;
        //     origin_pointcloud.y = cloud_ptr->points[initial_index].y;
        //     origin_pointcloud.z = cloud_ptr->points[initial_index].z;
        //     origincloudptr_to_ROSMsg->points.push_back(origin_pointcloud);

		// 	//printf("origin_pointcloud.x : %d\n", cloud_ptr->points[initial_index].x);

		// 	//origin_point.data.push_back(cloud_ptr->points[initial_index].x);
		// 	//origin_point.data.push_back(cloud_ptr->points[initial_index].y);
		// 	//origin_point.data.push_back(cloud_ptr->points[initial_index].z);
		// 	//origin_point_pub.publish(origin_point);
		// 	//origin_point.data.clear();

        // }
		

        // int target_index = 640 * points[1][i].y + points[1][i].x; //計算Target point單一pixel-wise於640x480第幾個
        // if(pcl_isfinite(cloud_ptr->points[target_index].x))
        // {
		// 	//ROS_INFO("\033[1;31m\nTarget point[x y z] : [%lf %lf %lf]\n\033[0m",cloud_ptr->points[target_index].x,cloud_ptr->points[target_index].y,cloud_ptr->points[target_index].z);
        //     //cout << "Target point:\nx: " << cloud_ptr->points[target_index].x;
        //     //cout << "\ny: " << cloud_ptr->points[target_index].y;
        //     //cout << "\nz: " << cloud_ptr->points[target_index].z << endl;
        //     //cout << "**************" << endl;
        //     //cout << endl;
            
        //     // target_pointcloud.r = cloud_ptr->points[target_index].r;
        //     // target_pointcloud.g = cloud_ptr->points[target_index].g;
        //     // target_pointcloud.b = cloud_ptr->points[target_index].b;
        //     target_pointcloud.r = 255;
        //     target_pointcloud.g = 0;
        //     target_pointcloud.b = 0;
        //     target_pointcloud.x = cloud_ptr->points[target_index].x;
        //     target_pointcloud.y = cloud_ptr->points[target_index].y;
        //     target_pointcloud.z = cloud_ptr->points[target_index].z;
        //     targetcloudptr_to_ROSMsg->points.push_back(target_pointcloud);

		// 	//target_point.data.push_back(cloud_ptr->points[target_index].x);
		// 	//target_point.data.push_back(cloud_ptr->points[target_index].y);
		// 	//target_point.data.push_back(cloud_ptr->points[target_index].z);
		// 	//target_point_pub.publish(target_point);
		// 	//target_point.data.clear();

        // }
		
		// if((pcl_isfinite(cloud_ptr->points[initial_index].x)) && (pcl_isfinite(cloud_ptr->points[target_index].x)))
        // {
		// 	x_op_original = cloud_ptr->points[initial_index].x;
		// 	y_op_original = cloud_ptr->points[initial_index].y;
		// 	z_op_original = cloud_ptr->points[initial_index].z;
		// 	x_op_target = cloud_ptr->points[target_index].x;
		// 	y_op_target = cloud_ptr->points[target_index].y;
		// 	z_op_target = cloud_ptr->points[target_index].z;
		// 	ROS_INFO("\033[1;32mOptical Flow points : [ %lf %lf %lf %lf %lf %lf ]\033[0m",x_op_original,y_op_original,z_op_original,x_op_target,y_op_target,z_op_target);
		// 	point_data.data.push_back(cloud_ptr->points[initial_index].x);
		// 	point_data.data.push_back(cloud_ptr->points[initial_index].y);
		// 	point_data.data.push_back(cloud_ptr->points[initial_index].z);
		// 	point_data.data.push_back(cloud_ptr->points[target_index].x);
		// 	point_data.data.push_back(cloud_ptr->points[target_index].y);
		// 	point_data.data.push_back(cloud_ptr->points[target_index].z);
		// 	point_data_pub.publish(point_data);
		// 	point_data.data.clear();
			
		// 	ROS_INFO("\033[1;31mDistance : [ %lf m ]\033[0m",ComputeDistance(x_op_original,y_op_original,z_op_original,x_op_target,y_op_target,z_op_target));
		// 	distance_data.data.push_back(distance);
		// 	distance_data_pub.publish(distance_data);
		// 	distance_data.data.clear();

		// 	ROS_INFO("\033[1;33mcmd_vel : [ %lf m/s ]\n\033[0m",ComputeVelocity(distance, delta_time));
		// 	vel_data.data.push_back(vel);
		// 	vel_data_pub.publish(vel_data);
		// 	vel_data.data.clear();
		// }
//*******************************************************************************************************************************
    }
//**********************************************************************************************
    // pcl::toROSMsg(*(origincloudptr_to_ROSMsg), originPointCloudtoROSMsg);
    // originPointCloudtoROSMsg.header.frame_id = "map";
    // originPointCloudtoROSMsg.header.stamp = ros::Time::now();
    // origin_pointcloud_pub.publish(originPointCloudtoROSMsg);
    // origincloudptr_to_ROSMsg->points.clear();

    // pcl::toROSMsg(*(targetcloudptr_to_ROSMsg), targetPointCloudtoROSMsg);
    // targetPointCloudtoROSMsg.header.frame_id = "map";
    // targetPointCloudtoROSMsg.header.stamp = ros::Time::now();
    // target_pointcloud_pub.publish(targetPointCloudtoROSMsg);
    // targetcloudptr_to_ROSMsg->points.clear(); // ros::Duration(0.033).sleep();
    // 把當前跟蹤結果作為下一此參考
//**********************************************************************************************
    swap(points[1], points[0]);
    swap(gray_prev, gray);
    //cv::imshow("Lucas–Kanade Optical Flow Tracking", output);
	output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
	output_image_pub.publish(output_image);
}


//决定哪些跟蹤點被接受
// return: 是否被添加標誌
bool LK_OpticalFlow::AddNewPoints()
{
    return points[0].size() <= 10;
}

//决定哪些跟蹤點被接受
bool LK_OpticalFlow::AcceptTrackedPoint(int i)
{
    deltaDist = abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y);
    // printf("\033[1;31m deltaDist : %f\n \033[0m", deltaDist);
	return status[i] && deltaDist > 2;
    // return status[i] && deltaDist > 100;// 50
}

// float LK_OpticalFlow::EuclideanDistance(int j)
// {
// 	EuclideanDis = sqrt(pow((points[0][i].x - points[1][i].x),2) + pow((points[0][i].y - points[1][i].y),2));
// 	// printf("\033[1;31m EuclideanDis : %f\n \033[0m", EuclideanDis);
// 	// return EuclideanDis;
// }

float LK_OpticalFlow::EuclideanDistance(float init_x, float init_y, float end_x, float end_y)
{
	EuclideanDis = sqrt(pow((end_x - init_x),2) + pow((end_y - init_y),2));
	// printf("\033[1;31m EuclideanDis : %f\n \033[0m", EuclideanDis);
	// return EuclideanDis;
}

float LK_OpticalFlow::D4Distance(float init_x, float init_y, float end_x, float end_y)
{
	D4Dis = abs(end_x - init_x) + abs(end_y - init_y);
	// printf("\033[1;34m D4Dis : %f\n \033[0m", D4Dis);
	// return D4Dis;
}

// float LK_OpticalFlow::ComputeDistance(float x_o, float y_o, float z_o, float x_t, float y_t, float z_t)
// {
// 	//distance = sqrt(pow((x_t - x_o),2) + pow((y_t - y_o),2) + pow((z_t - z_o),2));
// 	distance = sqrt(pow((x_t - x_o),2) + pow((y_t - y_o),2));
// 	return distance;
// }

// float LK_OpticalFlow::ComputeVelocity(float dis, clock_t t)
// {
// 	//clock_t ts = (double)t/CLOCKS_PER_SEC;
// 	//topic_t = 1/15.725;
// 	vel = dis/t;
// 	return vel;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LKOpticalFlow_Realsense");
    ros::NodeHandle nh;
    LK_OpticalFlow LK_OpticalFlow(nh);
    ros::spin();
}
