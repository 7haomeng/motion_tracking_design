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
#include <opencv2/imgcodecs.hpp>

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

void RemoveOutlier(Mat RGB_frame, Mat mask_frame){
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
    Mat mImGray,predict_mImGray,maskImGray;
    Mat outimg1,predict_outimg,outimg2;
    vector<KeyPoint> keypoints_1, predict_keypoints_1, keypoints_2;
    Point2f outlier_points;
    Mat descriptors_1, predict_descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;
    // cout << "ok" << endl;
    assert(RGB_frame.data != nullptr);
    mImGray = RGB_frame;
    cvtColor(mImGray, mImGray, CV_RGB2GRAY);

    assert(mask_frame.data != nullptr);
    maskImGray = mask_frame;
    cvtColor(maskImGray, maskImGray, CV_RGB2GRAY);

    cols = maskImGray.cols;
    rows = maskImGray.rows;
    channels = maskImGray.channels();

    detector = ORB::create();
    descriptor = ORB::create();
    detector->detect(mImGray, keypoints_1);
    descriptor->compute(mImGray, keypoints_1, descriptors_1);

    for(vector<KeyPoint>::iterator it = keypoints_1.begin(); it != keypoints_1.end(); ++it){
        cnt++;
        // cout << cnt << "." << &(*it) <<endl;
        // cout << cnt << ". Keypoints : " << (*it).pt << endl;
        check = true;
        for (r = 0; r <rows; r++){
            if (check == false){
                break;
            }else{
                data = maskImGray.ptr<uchar>(r);
                for(c = 0; c < cols * maskImGray.channels(); c++){
                    if (check == false){
                        break;
                    }else{
                        index = 640 * r + c;
                        mask_label = int(data[c]);
                        if(mask_label == 255){
                            index_x = c;
                            index_y = r;
                            // cout << cnt << ". Pixel [x,y] : [" << index_x << "," << index_y << "]" << endl;
                            if ((index_x == (int)((*it).pt.x)) && index_y == (int)((*it).pt.y)){
                                // cout << "okok" << endl;
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

    drawKeypoints(RGB_frame, keypoints_1, outimg1, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    imshow("RGB Image", outimg1);
    waitKey(0);
}

int main(int argc, char** argv){
    Mat image = imread("/home/hao/motion_tracking_design/src/orbtest/src/frame1299.jpg", 1);
    Mat mask = imread("/home/hao/motion_tracking_design/src/orbtest/src/frame1299.png");

    Mat element;
    element = getStructuringElement(MORPH_RECT, Size(8, 8));
    dilate(mask, mask, element);

    RemoveOutlier(image, mask);

    // imshow("RGB Image", mask);

    waitKey(0);
	return 0;
}