#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


using namespace ros;
using namespace std;
using namespace cv;

class Moving_Check{
    private:
    int i,j;
    int rows, cols;
    Mat frame;

    public:
    Moving_Check(ros::NodeHandle);
    void mask_img_Callback(const sensor_msgs::ImageConstPtr&);
    image_transport::Subscriber mask_img_sub;

};

Moving_Check::Moving_Check(ros::NodeHandle nh){
    image_transport::ImageTransport it(nh);
    mask_img_sub = it.subscribe("/ESPNet_v2/mask_img", 1, &Moving_Check::mask_img_Callback,this);
}

void Moving_Check::mask_img_Callback(const sensor_msgs::ImageConstPtr& mask_msg){
    frame = cv_bridge::toCvCopy(mask_msg, "bgr8")->image;
    cout << "Size:" << frame.size() << " " << "Column: " << frame.cols << " " << "Row: " << frame.rows << endl;

    cols = frame.cols;
    rows = frame.rows;

    for (i = 0; i <cols; i++){
        for (j = 0; j<rows; j++){
            // TODO: find the value is 1;
        }
    }
        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Moving_Check");
    ros::NodeHandle nh;
    Moving_Check Moving_Check(nh);
    ros::spin();
}