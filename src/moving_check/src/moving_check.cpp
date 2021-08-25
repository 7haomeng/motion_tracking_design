#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <math.h>

using namespace ros;
using namespace std;

class Moving_Check{
    private:

    public:
    Moving_Check(ros::NodeHandle);

};

Moving_Check::Moving_Check(ros::NodeHandle nh){
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Moving_Check");
    ros::NodeHandle nh;
    Moving_Check Moving_Check(nh);
    ros::spin();
}