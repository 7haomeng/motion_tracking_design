#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
using namespace ros;
using namespace std;

int i = 0;
std::ofstream myfile;
std::ofstream distance_myfile;
std::ofstream vel_myfile;

void point_data_csv(const std_msgs::Float64MultiArray::ConstPtr& point_msg){
	//std::ofstream myfile;
	myfile.open ("/home/graduationv2/DynamicTracking-LKOpticalFlow/src/LKOpticalFlow/csv/point_data.txt", std::ios_base::app);
	//myfile << ",origin_point.x,origin_point.y,origin_point.z,target_point.x,target_point.y,target_point.z\n";
	//myfile << "Data no." << i << ",";
	for(int k = 0; k < point_msg->data.size(); k++){
		if(!isnan(point_msg->data[k])){
			myfile << point_msg->data[k] << ",";
		}
	}
	i++;
	myfile << "\n";
    myfile.close();
}

void distance_data_csv(const std_msgs::Float64MultiArray::ConstPtr& point_msg){
	//std::ofstream myfile;
	distance_myfile.open ("/home/graduationv2/DynamicTracking-LKOpticalFlow/src/LKOpticalFlow/csv/distance_data.txt", std::ios_base::app);
	//myfile << ",origin_point.x,origin_point.y,origin_point.z,target_point.x,target_point.y,target_point.z\n";
	//myfile << "Data no." << i << ",";
	for(int j = 0; j < point_msg->data.size(); j++){
		if(!isnan(point_msg->data[j])){
			distance_myfile << point_msg->data[j];
		}
	}
	i++;
	distance_myfile << "\n";
    distance_myfile.close();
}

void vel_data_csv(const std_msgs::Float64MultiArray::ConstPtr& point_msg){
	//std::ofstream myfile;
	vel_myfile.open ("/home/graduationv2/DynamicTracking-LKOpticalFlow/src/LKOpticalFlow/csv/vel_data.txt", std::ios_base::app);
	//myfile << ",origin_point.x,origin_point.y,origin_point.z,target_point.x,target_point.y,target_point.z\n";
	//myfile << "Data no." << i << ",";
	for(int a = 0; a < point_msg->data.size(); a++){
		if(!isnan(point_msg->data[a])){
			vel_myfile << point_msg->data[a];
		}
	}
	i++;
	vel_myfile << "\n";
    vel_myfile.close();
}

int main( int argc, char* argv[] ){
	ros::init(argc, argv, "point_data_csv");
    ros::NodeHandle nh;
	ros::Subscriber point_sub = nh.subscribe("lk/point_data", 1000, point_data_csv);	
	ros::Subscriber distance_sub = nh.subscribe("lk/distance_data", 1000, distance_data_csv);
	ros::Subscriber vel_sub = nh.subscribe("lk/vel_data", 1000, vel_data_csv);

    ros::spin();
	return 0;
}
