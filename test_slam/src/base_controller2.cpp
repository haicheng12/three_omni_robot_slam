#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

// Signal handling
#include <signal.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

#include "ros/assert.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace boost::asio;

ros::Time current_time, last_time;

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dt = 0.0;

union _SPEED_
{
   uint8_t speed_buf[41];
   struct _speed_value_
   {
       float correct_x;
       float correct_y;
       float correct_z;
       float slam_vx;
       float slam_vy;
       float slam_vz;
       float remote_x;
       float remote_y;
       float remote_z;
       char slam_remote_change;
       uint32_t CRC;
   }Struct_Speed;
}Union_Speed;

uint32_t CRC_verify(uint8_t* data,uint8_t len)
{
    uint32_t CRC=0;
    uint8_t CIR1,CIR2;
    for(CIR2=0;CIR2<8;CIR2++)
    {
        for(CIR1=0;CIR1<len-4;CIR1++)
        {
             CRC= ((data[CIR1]>>CIR2)&0x01)*(CIR1+1)*(CIR2+1)+CRC;
        }
    }
    return CRC;
}

union _Data_
{
    uint8_t buf_rev[16];
    struct _value_velocity_
    {
        float corrent_vx;
        float corrent_vy;
        float corrent_vth;
       uint32_t  crc;
    }Struct_Velocity;
}Union_Data;

uint32_t CRC3;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    geometry_msgs::Twist twist = twist_aux;
    Union_Speed.Struct_Speed.slam_vx = twist_aux.linear.y*0.5;
    Union_Speed.Struct_Speed.slam_vy = twist_aux.linear.x*(-0.5);
    Union_Speed.Struct_Speed.slam_vz = twist_aux.angular.z*(-0.15);

    Union_Speed.Struct_Speed.slam_remote_change= 0;
    CRC3 =    CRC_verify(Union_Speed.speed_buf,41);
    Union_Speed.Struct_Speed.CRC = CRC3;
}

void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped& msg )
{
   ROS_INFO_STREAM(std::setprecision(2) << std::fixed
                   << "position=  ("<< msg.pose.pose.position.x<<","
                   << msg.pose.pose.position.y <<")"
                   <<" *direction="<< msg.pose.pose.orientation.z);

   Union_Speed.Struct_Speed.correct_x = msg.pose.pose.position.x;
   Union_Speed.Struct_Speed.correct_y = msg.pose.pose.position.y;
   Union_Speed.Struct_Speed.correct_z = msg.pose.pose.orientation.z;

   Union_Speed.Struct_Speed.slam_remote_change= 1;
   CRC3 =    CRC_verify(Union_Speed.speed_buf,41);
   Union_Speed.Struct_Speed.CRC = CRC3;

}

void positionCallback(const geometry_msgs::Twist &twist_aux)
{
    geometry_msgs::Twist twist = twist_aux;

    switch((short)twist_aux.linear.z)
    {
      case 0:
        Union_Speed.Struct_Speed.remote_x=0.0;//-y_position
        Union_Speed.Struct_Speed.remote_y=0.0;//-x_position
        Union_Speed.Struct_Speed.remote_z=0.0;//
      break;

      case 1:
        Union_Speed.Struct_Speed.remote_x=-0.5;//-y_position(0.5,0.5,0)
        Union_Speed.Struct_Speed.remote_y=-0.5;//-x_position
        Union_Speed.Struct_Speed.remote_z=1.57;//
      break;

      case 2:
        Union_Speed.Struct_Speed.remote_x=0.5;//-y_position(0.5,-0.5,0)
        Union_Speed.Struct_Speed.remote_y=-0.5;//-x_position
        Union_Speed.Struct_Speed.remote_z=-1.57;
      break;

      case 3:
        Union_Speed.Struct_Speed.remote_x=4.0;//连续的目标点
        Union_Speed.Struct_Speed.remote_y=4.0;
        Union_Speed.Struct_Speed.remote_z=0.0;

        Union_Speed.Struct_Speed.remote_x=5.0;
        Union_Speed.Struct_Speed.remote_y=5.0;
        Union_Speed.Struct_Speed.remote_z=0.0;

        Union_Speed.Struct_Speed.remote_x=6.0;
        Union_Speed.Struct_Speed.remote_y=6.0;
        Union_Speed.Struct_Speed.remote_z=0.0;
      break;
    }

    Union_Speed.Struct_Speed.slam_remote_change= 1;
    CRC3 =    CRC_verify(Union_Speed.speed_buf,41);
    Union_Speed.Struct_Speed.CRC = CRC3;

}

int main(int argc, char** argv)
{
    unsigned char check_buf[1];
    unsigned char i;

    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle node_Teleop;
    ros::NodeHandle node_Pose;
    ros::NodeHandle node_Position;

    ros::Subscriber teleop_sub = node_Teleop.subscribe("/cmd_vel", 100, cmd_velCallback);
    ros::Subscriber pose_sub = node_Pose.subscribe("/amcl_pose",100,&poseMessageReceived);
    ros::Subscriber position_sub = node_Position.subscribe("/cmd_vel", 100, positionCallback);

    boost::system::error_code errsend,errtx;
    ros::Publisher odom_pub = node_Teleop.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    const double degree = M_PI/180;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while(ros::ok())
    {
        ros::spinOnce();

        read(sp, buffer(Union_Data.buf_rev));

        if(CRC_verify(Union_Data.buf_rev,16)==Union_Data.Struct_Velocity.crc)
        {
            current_time = ros::Time::now();

            x = -Union_Data.Struct_Velocity.corrent_vx/1000/1000;
            y = Union_Data.Struct_Velocity.corrent_vy/1000/1000;
            th = Union_Data.Struct_Velocity.corrent_vth*M_PI/180*M_PI/180;

            if(th > M_PI)
                th = th - 2 * M_PI;
            if(th < - M_PI)
                th = th + 2 * M_PI;

//            ROS_INFO("x: %2f",x);
//            ROS_INFO("y: %2f",y);
//            ROS_INFO("th: %2f", th);

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation = odom_quat;

            odom.twist.twist.linear.x = Union_Speed.Struct_Speed.slam_vx;
            odom.twist.twist.linear.y = Union_Speed.Struct_Speed.slam_vy;
            odom.twist.twist.linear.z = 0.0;
            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = Union_Speed.Struct_Speed.slam_vz;

            last_time = ros::Time::now();
            odom_broadcaster.sendTransform(odom_trans);

            odom_pub.publish(odom);
       }

        else
        {
            ROS_INFO("Fucking communication fails,The fuck can I hurry up to restart!");
            read(sp, buffer(check_buf));
        }

        write(sp, buffer(Union_Speed.speed_buf,41));
        last_time = current_time;
    }
    iosev.run();
}
