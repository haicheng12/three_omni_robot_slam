#include "ros/ros.h"
#include "iostream"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <ctime>
#include <unistd.h>

using namespace std;
using namespace boost;

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
union _rxvel  recevel;

int main(int argc,char**argv)
{
    ros::init(argc,argv,"encoder_odom");
    size_t ret ;
    boost::asio::io_service io;
    boost::asio::serial_port sp(io, "/dev/ttyUSB0");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    boost::system::error_code errsend,errtx;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
       ros::Time current_time = ros::Time::now();
       ros::spinOnce();

       ret = sp.read_some(boost::asio::buffer(recevel.num, 16), errtx);

       if(CRC_verify(recevel.num,16)==recevel.velocity.crc)
       {
           ROS_INFO("x: %2f",recevel.velocity.x_position);
           ROS_INFO("y: %2f",recevel.velocity.y_position);
           ROS_INFO("th: %2f",recevel.velocity.th_position);
       }
       ros::Time last_time = current_time;

       loop_rate.sleep();
    }

}

