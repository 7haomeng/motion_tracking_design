#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>
#include <math.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Mat img_rgb = imread("/home/chinghaomeng/motion_tracking_design/src/segmentation/src/dataSet/hao/images/frame0000.jpg");

    for (int r = 0; r < img_rgb.rows; r++){
        for (int c = 0; c < img_rgb.cols; c++){
            int index = 640 * r + c;
            cout << "Index: " << index << "," << "Pixel at position (x, y): (" << c << "," << r << ") = " << img_rgb.at<Vec3b>(r,c) << endl;
    
        }

    }
}