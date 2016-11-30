//
// Created by yyq on 16-11-24.
//

#ifndef TURTLE_CONTROL_TURTLE_CONTROL_H
#define TURTLE_CONTROL_TURTLE_CONTROL_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>

using namespace cv;

class Turtle_control
{
public:
    Turtle_control(vector<Point2f> target)
    {
        sub_=nh_.subscribe("odom",1000,&Turtle_control::motion_control,this);

        pub_=nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",1000);

        target_=target;
    }
    void motion_control(const nav_msgs::Odometry msg);

    int motion_plan(nav_msgs::Odometry msg,int num,double roll);

    void motion_tf();

    void motion_circle(nav_msgs::Odometry msg,double roll);

    double normalize_angle(double angle);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    tf::TransformListener listener_;
    vector<Point2f> target_;
    double w_,x_,y_,z_;
    double yaw_,pitch_,roll_;
    double position_x,position_y,position_z;
    double vel_x,angular_z;
    double distance_;
    double turn_angle,delta_angle,last_angle,start_angle;
    double x_start,y_start;
};
#endif //TURTLE_CONTROL_TURTLE_CONTROL_H

