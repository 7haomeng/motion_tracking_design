#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <fstream>


using namespace ros;
using namespace std;
using namespace cv;

class Moving_Check{
    private:
    int i,j;
    size_t rows, cols;
    int channels;
    Mat frame;
    Vec3b PixelValue;
    uint8_t* pixelPtr;
    int cn;
    Scalar_<uint8_t> bgrPixel;

    sensor_msgs::ImagePtr output_tstimage;

    public:
    Moving_Check(ros::NodeHandle);
    void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    image_transport::Subscriber mask_img_sub;
    image_transport::Publisher output_tstimg_pub;

};

Moving_Check::Moving_Check(ros::NodeHandle nh){
    image_transport::ImageTransport it(nh);
    mask_img_sub = it.subscribe("/ESPNet_v2/mask_img", 1, &Moving_Check::mask_img_Callback,this);
    output_tstimg_pub = it.advertise("moving_check/tst_image", 1);
}

void Moving_Check::mask_img_Callback(const sensor_msgs::ImageConstPtr& mask_msg){
    frame = cv_bridge::toCvCopy(mask_msg, "8UC1")->image;
    cv::Mat dstImage(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));
    dstImage = frame.clone();
    // cout << frame << endl;
    // cout << "**********************************************************" << endl;
    // cout << "Size:" << frame.size() << " " << "Column: " << frame.cols << " " << "Row: " << frame.rows << endl;

    cols = frame.cols;
    rows = frame.rows;
    channels = frame.channels();

    cout << "Mask_img: " << endl;
    for (i = 0; i <rows; i++){
        uchar* data = frame.ptr<uchar>(i);
        for(j = 0; j < cols * frame.channels(); j++){
            int index = 640 * i + j;
            int mask_label = int(data[j]);
            if(mask_label == 1){
                cout << "000" << endl;
                dstImage.at<uchar>(i, j) = 255;
                cout << "111" << endl;
                // cout << "Index: " << index << endl;
            }
            // cout << int( data[j] );
        }
        // cout << " " << endl;
    }
    output_tstimage = cv_bridge::CvImage(std_msgs::Header(), "8UC1", dstImage).toImageMsg();
	output_tstimg_pub.publish(output_tstimage);
        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Moving_Check");
    ros::NodeHandle nh;
    Moving_Check Moving_Check(nh);
    ros::spin();
}