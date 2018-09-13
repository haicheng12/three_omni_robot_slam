#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <ctime>

using namespace std;

double dt = 0.0;
double imu_z = 0.0;

ros::Time current_time,last_time;

void IMUCallback(const sensor_msgs::Imu& imu)
{
    //callback every time the robot's angular velocity is received
    current_time = ros::Time::now();

    imu_z = imu.angular_velocity.z*100;

    dt = (current_time - last_time).toSec();
    last_time = current_time;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"imu_odom");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 100, IMUCallback);
    ros::Rate loop_rate(10);

    double theta = 0.0;
    double angle = 0.0;

    while(ros::ok())
    {
       current_time = ros::Time::now();
       ros::spinOnce();

       //angular velocity is the rotation in Z from imu_filter_madgwick's output
       double angular_velocity = imu_z;

       //calculate angular displacement  θ = ω * t
       double delta_theta = angular_velocity * dt; //radians

       //calculate current position of the robot
       theta += delta_theta;
       angle=theta*180/M_PI;

       ROS_INFO("theta: %2f",theta);
       ROS_INFO("angle: %2f",angle);

       last_time = current_time;

       loop_rate.sleep();
    }

}

