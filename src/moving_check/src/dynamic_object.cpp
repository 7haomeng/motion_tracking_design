#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/core.hpp>

#include <sensor_msgs/Image.h>

#include <cv.h>
#include <highgui.h>

using namespace ros;
using namespace std;
using namespace cv;

class Dynamic_Object{
    private:
    int i,j,k;
    int dx, dy;
    int p = 0;
    int rows, cols;
    int const MAX_CORNERS = 1000;
    string window_name = "dynamic object";
    cv_bridge::CvImagePtr cv_ptr; 
    Mat frame;
    Mat result;
    // Mat move_img;
    Mat gray;   // 當前圖片
    Mat gray_prev;  // 預測圖片

    IplImage *src_img1;  //the previous frame (gray)
    IplImage *src_img2;  //the current frame(gray)

    IplImage *dst_img;   //the result
    IplImage *cur_img;   
    IplImage *pre_img;

    IplImage prev_tmp;
    IplImage tmp;
    IplImage moving_tmp;
    IplImage pyrA_tmp;
    IplImage pyrB_tmp;

    CvArr* prev_arr;
    CvArr* arr;
    CvArr* moving_arr;
    CvArr* pyrA_arr;
    CvArr* pyrB_arr;

    CvPoint2D32f * move_old_point = new CvPoint2D32f[ MAX_CORNERS];
    CvPoint2D32f * move_new_point = new CvPoint2D32f[ MAX_CORNERS];

    char *features_found = new char[MAX_CORNERS];
    float *features_error = new float[MAX_CORNERS];

    CvTermCriteria criteria;

    sensor_msgs::ImagePtr output_image;

    public:
    Dynamic_Object(ros::NodeHandle);
    void dynamic_object_Callback(const sensor_msgs::ImageConstPtr&);
    void Tracking(Mat &);
    ros::Subscriber image_raw_sub;
    image_transport::Publisher output_image_pub;
};



Dynamic_Object::Dynamic_Object(ros::NodeHandle nh)
{ 
    image_raw_sub = nh.subscribe("/camera/color/image_raw", 1, &Dynamic_Object::dynamic_object_Callback,this);

    image_transport::ImageTransport it(nh);
	output_image_pub = it.advertise("dynamic_opencv_image", 1);
}

void Dynamic_Object::dynamic_object_Callback(const sensor_msgs::ImageConstPtr& image_msg){

    CvTermCriteria criteria;
    criteria = cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01);
    cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

    if(!frame.empty())
    {
        Tracking(frame);     
    }
    else
    { 
        printf("No captured frame -- Break!\n");
        return;
    }
}

void Dynamic_Object::Tracking(Mat &frame){

    cvtColor(frame, gray, CV_BGR2GRAY);
    // frame.copyTo(output);

    if (gray_prev.empty())
    {
        // cout << gray_prev <<endl;
        gray.copyTo(gray_prev);
    }

    Mat move_img(480, 640, CV_8UC1, Scalar(0,0,0));

    //cvAbsDiff(src_img1, src_img2,move_img);

    cols = gray_prev.cols; 
    rows = gray_prev.rows;

    for (i = 0; i <cols; i++)
    {
        for (j = 0; j<rows; j++)
        {
            prev_tmp = IplImage(gray_prev);
            prev_arr = (CvArr*)&prev_tmp;

            tmp = IplImage(gray);
            arr = (CvArr*)&tmp;

            moving_tmp = IplImage(move_img);
            moving_arr = (CvArr*)&moving_tmp;

            double a = abs(cvGet2D(prev_arr, j, i).val[0]-cvGet2D(arr, j, i).val[0]);
            // printf("prev_arr(B,G,R): (%f, %f, %f)\n", cvGet2D(prev_arr, j, i).val[0], cvGet2D(prev_arr, j, i).val[1], cvGet2D(prev_arr, j, i).val[2]);
            // printf("arr(B,G,R): (%f, %f, %f)\n", cvGet2D(arr, j, i).val[0], cvGet2D(arr, j, i).val[1], cvGet2D(arr, j, i).val[2]);
            // printf("Pixel Value: %f\n\n", a);
            CvScalar b = cvScalar(a, 0, 0,0);
            cvSet2D(moving_arr, j, i,b);

            if (a>40)
            {
                if (p<MAX_CORNERS-1)
                {
                    int d = ++p;
                    move_old_point[d].x = i;
                    move_old_point[d].y = j;
                }
            }
        }
    }

    CvSize Pyrsize = cvSize(gray_prev.cols + 8, gray_prev.rows/3);
    Mat pyrA(480, 640, CV_32FC1, Scalar(0,0,0));
    pyrA_tmp = IplImage(pyrA);
    pyrA_arr = (CvArr*)&pyrA_tmp;

    Mat pyrB(480, 640, CV_32FC1, Scalar(0,0,0));
    pyrB_tmp = IplImage(pyrB);
    pyrB_arr = (CvArr*)&pyrB_tmp;

    cvCalcOpticalFlowPyrLK(prev_arr, arr, pyrA_arr, pyrB_arr, move_old_point, move_new_point, MAX_CORNERS, cvSize(10,10), 3, features_found, features_error, criteria, 0);
    cout << "calculate optical flow" << endl;
    // calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);

    for(k = 0; k < MAX_CORNERS; k++){
        int x1 = (int)move_new_point[i].x;
        int x2 = (int)move_old_point[i].x;
        int y1 = (int)move_new_point[i].y;
        int y2 = (int)move_old_point[i].y;
        cout << "x1: " << x1 << " x2: " << x2 <<  " y1: " << y1 << " y2: " << y2 << endl;

        dx = (int)abs(x1 - x2);
        dy = (int)abs(y1 - y2);
        // if (dx >= 5 && dy >= 5){
            cvLine(moving_arr, cvPoint(x2,y2), cvPoint(x1,y1), CV_RGB(255,0,0), 3, CV_AA, 0);
        // }
    }

    swap(gray_prev, gray);
    // cout << cvarrToMat(moving_arr).size() << endl;
    // imshow(window_name, cvarrToMat(moving_arr));

    output_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", cvarrToMat(moving_arr)).toImageMsg();
	output_image_pub.publish(output_image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Dynamic_Object");
    ros::NodeHandle nh;
    Dynamic_Object Dynamic_Object(nh);
    ros::spin();
}