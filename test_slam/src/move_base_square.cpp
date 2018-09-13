#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

using namespace std;
using namespace ros;

Publisher cmdVelPub;//创建两个发布者
Publisher marker_pub;

void shutdown(int sig)//停止的回调函数
{
  cmdVelPub.publish(geometry_msgs::Twist());
  Duration(1).sleep(); // sleep for  a second
  ROS_INFO("nav_square.cpp ended!");

  shutdown();
}

void init_markers(visualization_msgs::Marker *marker)//标识函数
{
  marker->ns       = "waypoints";//各种参数
  marker->id       = 0;
  marker->type     = visualization_msgs::Marker::CUBE_LIST;//类型
  marker->action   = visualization_msgs::Marker::ADD;//行为
  marker->lifetime = ros::Duration();//0 is forever 离开时间
  marker->scale.x  = 0;//这里应该是初始原点x和y的位置
  marker->scale.y  = 0;
  marker->color.r  = 1.0;//颜色
  marker->color.g  = 0.7;
  marker->color.b  = 1.0;
  marker->color.a  = 1.0;

  marker->header.frame_id = "odom";
  marker->header.stamp = ros::Time::now();//标志
}

int main(int argc, char** argv)
{
  init(argc, argv, "nav_move_base");

  string topic = "/cmd_vel";
  NodeHandle node;

  //Subscribe to the move_base action server 订阅move_base行为
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

  //Define a marker publisher.定义一个标识的发布者
  marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

  //for init_markers function 初始化标记的功能
  visualization_msgs::Marker  line_list;

  signal(SIGINT, shutdown);//信号
  ROS_INFO("move_base_square.cpp start...");

  //How big is the square we want the robot to navigate? 机器人驾驶的面积大小
  double square_size = 2.0;

  //Create a list to hold the target quaternions (orientations)列表来保持四元数，我也不知道什么东西
  geometry_msgs::Quaternion quaternions[4];

  //convert the angles to quaternions 四元素包含的角度
  double angle = M_PI/2;

  int angle_count = 0;
  for(angle_count = 0; angle_count < 4;angle_count++ )//这个函数是让角度每次转90度，转四次为一个循环
  {
      quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
      angle = angle + M_PI/2;
  }

  //a pose consisting of a position and orientation in the map frame.在地图的位置和角度的点
  geometry_msgs::Point point;//定义一个消息类型的点
  geometry_msgs::Pose pose_list[4];

  point.x = square_size;//x坐标长度是正方形边长大小
  point.y = 0.0; //y坐标为0
  point.z = 0.0;//角度为0
  pose_list[0].position = point;
  pose_list[0].orientation = quaternions[0];

  point.x = square_size;
  point.y = square_size;
  point.z = 0.0;
  pose_list[1].position = point;
  pose_list[1].orientation = quaternions[1];

  point.x = 0.0;
  point.y = square_size;
  point.z = 0.0;
  pose_list[2].position = point;
  pose_list[2].orientation = quaternions[2];

  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  pose_list[3].position = point;
  pose_list[3].orientation = quaternions[3];

  //Initialize the visualization markers for RViz 初始化视图
  init_markers(&line_list);

  //Set a visualization marker at each waypoint 设置一个可视化的标识到每一个路径
  for(int i = 0; i < 4; i++)
  {
    line_list.points.push_back(pose_list[i].position);
  }

  //Publisher to manually control the robot (e.g. to stop it, queue_size=5) 手动地发布控制
  cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 5);

  ROS_INFO("Waiting for move_base action server...");

  //Wait 60 seconds for the action server to become available 每次行为服务变为可行的时候等待5秒
  if(!ac.waitForServer(ros::Duration(5)))
  {
    ROS_INFO("Can't connected to move base server");
    return 1;
  }

  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");

  //Initialize a counter to track waypoints 初始化一个计数器来追踪目标点
  int count = 0;

  //Cycle through the four waypoints
  while( (count < 4) && (ros::ok()) )
  {
     //Update the marker display 更新标识物展示
     marker_pub.publish(line_list);

     //Intialize the waypoint goal  初始化目标点目标
     move_base_msgs::MoveBaseGoal goal;

     //Use the map frame to define goal poses 使用地图框架去定义目标姿势
     goal.target_pose.header.frame_id = "map";

     //Set the time stamp to "now" 更新时间
     goal.target_pose.header.stamp = ros::Time::now();

     //Set the goal pose to the i-th waypoint 把目标形成路径
     goal.target_pose.pose = pose_list[count];

     //Start the robot moving toward the goal
     //Send the goal pose to the MoveBaseAction server 发送目标
     ac.sendGoal(goal);

    //Allow 1 minute to get there 允许一分钟到达那里
    bool finished_within_time = ac.waitForResult(ros::Duration(60));

    //If we dont get there in time, abort the goal 如果没有到达，输出没到达的信息
    if(!finished_within_time)
    {
        ac.cancelGoal();
        ROS_INFO("Timed out achieving goal");
    }

    else
    {
        //We made it! 我们做到了
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal succeeded!");
        }
        else
        {
            ROS_INFO("The base failed for some reason");
        }
    }
     count += 1; //计数器+1
  }

  ROS_INFO("move_base_square.cpp end...");

  return 0;

}

