#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>//ros头文件
#include <geometry_msgs/Twist.h>//消息头文件
#define KEYCODE_A 0x61  //键盘按键宏定义
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_H 0x68

#define KEYCODE_B 0x62

class TeleopPR2Keyboard //定义键盘的类
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;//定义行走速度，跑的速度，旋转速率，旋转时候跑的速率
  geometry_msgs::Twist cmd; //真实速度
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;

  public:
  void init()
  {
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0; //初始速度
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);//发布的主题
    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 0.5);//这里给各个定义的东西参数配置
    n_private.param("run_vel", run_vel, 1.0);
    n_private.param("yaw_rate", yaw_rate,1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 1.5);
  }
  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");//定义一个主题
  TeleopPR2Keyboard tpk;
  tpk.init();//初始化
  signal(SIGINT,quit);
  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;
  bool dirty=false;
  // get the console in raw mode 获得控制在一个新的模式
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file 设置新的线，然后结束文件
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");
  puts("---------------------------");
  puts("Press 'F G H' to arrive in 'A B C' position");
  for(;;)
  {
    // get the next event from the keyboard 从键盘中获得下一个事件
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    switch(c) //选择
    {
      // Walking
    case KEYCODE_W:
      cmd.linear.x = walk_vel;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.linear.x = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.linear.y = walk_vel;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.linear.y = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.angular.z = yaw_rate;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.angular.z = - yaw_rate;
      dirty = true;
      break;
      // Running
    case KEYCODE_W_CAP:
      cmd.linear.x = run_vel;
      dirty = true;
      break;
    case KEYCODE_S_CAP:
      cmd.linear.x = - run_vel;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      cmd.linear.y = run_vel;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      cmd.linear.y = - run_vel;
      dirty = true;
      break;
    case KEYCODE_Q_CAP:
      cmd.angular.z = yaw_rate_run;
      dirty = true;
      break;
    case KEYCODE_E_CAP:
      cmd.angular.z = - yaw_rate_run;
      dirty = true;
      break;

    case KEYCODE_F:
      cmd.linear.z=0.0;
      dirty = true;
      break;
    case KEYCODE_G:
      cmd.linear.z=1.0;
      dirty = true;
      break;
    case KEYCODE_H:
      cmd.linear.z=2.0;
      dirty = true;
      break;

    case KEYCODE_B:
      cmd.linear.z=3.0;
      dirty = true;
      break;
    }
    if (dirty == true)
    {
      vel_pub_.publish(cmd);
    }
  }
}
