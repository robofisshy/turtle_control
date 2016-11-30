//
// Created by yyq on 16-11-25.
//
#include "Turtle_control.h"

#define goal_distance 1
#define pi      3.141592653
#define goal_angle   pi/2-0.15

int start_=0;
void Turtle_control::motion_control(const nav_msgs::Odometry msg)
{
    int i=0;

    w_=msg.pose.pose.orientation.w;
    x_=msg.pose.pose.orientation.x;
    y_=msg.pose.pose.orientation.y;
    z_=msg.pose.pose.orientation.z;

    yaw_=atan2(2*(w_*x_+y_*z_),1-2*(x_*x_+y_*y_));          //Transform Quaternion to Eular-angle
    pitch_=asin(2*(w_*y_-z_*x_));
    roll_=atan2(2*(w_*z_+x_*y_),1-2*(y_*y_+z_*z_));

    vel_x=0.2;
    angular_z=0.7;
    if(start_==0){
        x_start=msg.pose.pose.position.x;
        y_start=msg.pose.pose.position.y;
        start_++;
        start_angle=roll_;
    }

    i=motion_plan(msg,i,roll_);
    //motion_circle(msg,roll_);
}

int Turtle_control::motion_plan(nav_msgs::Odometry msg,int num,double roll)
{
    geometry_msgs::Twist vel;
    int num1;
    ros::Rate rate_(30);
    position_x=msg.pose.pose.position.x;
    position_y=msg.pose.pose.position.y;
    if(num<3)
        num1=num+1;
    else
        num1=0;

    distance_=sqrt(pow((position_x-x_start),2)+pow((position_y-y_start),2));

    if (distance_<goal_distance)
    {
        vel.linear.x=0.2;
        //if(roll)
        vel.angular.z=0;
        pub_.publish(vel);
//        turn_angle=0;
//        last_angle=roll;
        start_angle=roll;
//        delta_angle=0;
        ROS_INFO_STREAM("Distance : " << distance_);
    }
    else if((distance_>=goal_distance)&&(normalize_angle(roll-start_angle)<goal_angle))
    {
        vel.angular.z=0.2;
        vel.linear.x=0;
//        delta_angle=normalize_angle(roll-last_angle);
//        turn_angle+=delta_angle;
        turn_angle=normalize_angle(roll-start_angle);
//        last_angle=roll;
        pub_.publish(vel);
        ROS_INFO_STREAM("Turn angle: " << roll <<" Last angle: "<<last_angle<<" Rotation angle"<<roll);
    }
    else if((distance_>=goal_distance)&&(normalize_angle(roll-start_angle)>=goal_angle))
    {
        x_start=position_x;
        y_start=position_y;
        turn_angle=0;
        return num1;
    }
    return num;
}
void Turtle_control::motion_tf()
{

}
void Turtle_control::motion_circle(nav_msgs::Odometry msg,double roll)
{
    geometry_msgs::Twist vel;
    if(roll-start_angle<goal_angle)
    {
        vel.angular.z=0.2;
        vel.linear.x=0;
        pub_.publish(vel);
        ROS_INFO_STREAM("Turn angle: " << roll );
    }
}

double Turtle_control::normalize_angle(double angle)
{
    double res=angle;
    if(res>pi)
        res-=2*pi;
    else if(res<-pi)
        res+=2*pi;
    return res;
}