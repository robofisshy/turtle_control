//
// Created by yyq on 16-11-22.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "Turtle_control.h"

using namespace std;
using namespace cv;

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"turtle_control_node");
    vector<Point2f> map(4);
    map[0]={0.0,0.0};
    map[1]={3.0,0.0};
    map[2]={3.0,3.0};
    map[3]={3.0,0.0};

    Turtle_control turtle_control(map);

    ros::spin();
    return 0;
}

