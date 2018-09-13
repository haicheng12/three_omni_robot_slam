#include "ros/ros.h"
#include "iostream"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <sensor_msgs/Imu.h>
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <ctime>
#include <unistd.h>

using namespace std;
using namespace boost;

double g_vel_dt = 0.0;
ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);

union _txvel
{
   uint8_t num[41];
   struct Vel
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
   }velocity;
};

uint32_t CRC_verify(uint8_t* data,uint8_t len)
{
    uint32_t CRC=0;
    uint8_t CIR11,CIR2;
    for(CIR2=0;CIR2<8;CIR2++)
    {
        for(CIR11=0;CIR11<len-4;CIR11++)
        {
            CRC= ((data[CIR11]>>CIR2)&0x01)*(CIR11+1)*(CIR2+1)+CRC;
        }
    }
    return CRC;
}

union _rxvel
{
   uint8_t num[16];
   struct Vel
   {
       float x_position;
       float y_position;
       float th_position;
       uint32_t  crc;
   }velocity;
};

uint32_t CRC1;
union _txvel  receiveVel;
union _rxvel  recevel;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();

    receiveVel.velocity.slam_vy = msg->linear.x*(-0.3);
    receiveVel.velocity.slam_vx = msg->linear.y*(0.3);
    receiveVel.velocity.slam_vz = msg->angular.z*(-0.1);
    receiveVel.velocity.slam_remote_change= 0;
    CRC1 = CRC_verify(receiveVel.num,41);
    receiveVel.velocity.CRC = CRC1;

    g_vel_dt = (current_time - g_last_vel_time).toSec();
    g_last_vel_time = current_time;
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"base_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    size_t ret ;
    boost::asio::io_service io;
    boost::asio::serial_port sp(io, "/dev/ttyUSB0");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::Subscriber sub =n.subscribe("cmd_vel",50,cmd_velCallback);
    boost::system::error_code errsend,errtx;
    ros::Publisher   pub = n.advertise<nav_msgs::Odometry>("odom",50);
    tf::TransformBroadcaster odom_broadcaster;
    double rate = 10.0;
    double x=0.0;
    double y=0.0;
    double th=0.0;

    ros::Rate r(rate);
    while(n.ok())
    {
       ros::spinOnce();
       ros::Time current_time = ros::Time::now();

       boost::asio::write(sp, boost::asio::buffer(receiveVel.num, 41));

       ret = sp.read_some(boost::asio::buffer(recevel.num, 16), errtx);

       if(CRC_verify(recevel.num,16)==recevel.velocity.crc)
       {
           ros::Time current_time = ros::Time::now();

           ROS_INFO("x: %2f",recevel.velocity.x_position);
           ROS_INFO("y: %2f",recevel.velocity.y_position);
           ROS_INFO("theta: %2f",recevel.velocity.th_position);

           x=recevel.velocity.x_position/1000;
           y=recevel.velocity.y_position/1000;
           th=recevel.velocity.th_position;

           if(th > M_PI)//添加导航时候的坐标角度限制，跟navigation的坐标一致
               th = th - 2 * M_PI;
           if(th < - M_PI)
               th = th + 2 * M_PI;

           //calculate robot's heading in quarternion angle
           //ROS has a function to calculate yaw in quaternion angle
           geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

           geometry_msgs::TransformStamped odom_trans;
           odom_trans.header.frame_id = "odom";
           odom_trans.child_frame_id = "base_link";
           //robot's position in x,y, and z
           odom_trans.transform.translation.x = x;
           odom_trans.transform.translation.y = y;
           odom_trans.transform.translation.z = 0.0;
           //robot's heading in quaternion
           odom_trans.transform.rotation = odom_quat;
           odom_trans.header.stamp = current_time;
           //publish robot's tf using odom_trans object
           odom_broadcaster.sendTransform(odom_trans);

           nav_msgs::Odometry odom;
           odom.header.stamp = current_time;
           odom.header.frame_id = "odom";
           //robot's position in x,y, and z
           odom.pose.pose.position.x = x;
           odom.pose.pose.position.y = y;
           odom.pose.pose.position.z = 0.0;
           //robot's heading in quaternion
           odom.pose.pose.orientation = odom_quat;

           odom.child_frame_id = "base_link";
           //linear speed from encoders
           odom.twist.twist.linear.x = receiveVel.velocity.slam_vx;
           odom.twist.twist.linear.y = receiveVel.velocity.slam_vy;
           odom.twist.twist.linear.z = 0.0;

           odom.twist.twist.angular.x = 0.0;
           odom.twist.twist.angular.y = 0.0;
           //angular speed from IMU
           odom.twist.twist.angular.z = receiveVel.velocity.slam_vz;

           //TODO: include covariance matrix here

           pub.publish(odom);

           g_last_loop_time = current_time;
           r.sleep();
       }
    }

}

