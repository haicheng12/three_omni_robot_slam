#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <cmath>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

MoveBaseClient* ac;

move_base_msgs::MoveBaseGoal goal;

void setPose(move_base_msgs::MoveBaseGoal &goal, double p_x, double p_y, double theta)
{
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = p_x;
    goal.target_pose.pose.position.y = p_y;
    goal.target_pose.pose.orientation.z = sin(theta/2);
    goal.target_pose.pose.orientation.w = cos(theta/2);

    ac->sendGoal(goal);
    ac->waitForResult();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_goals");

    ac = new MoveBaseClient("move_base", true);

    while (!ac->waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the move_base action server");
    }

    ROS_INFO("Sending goal");

    setPose(goal, 1.0, 0.0, 0.0);
    setPose(goal, 3.0, 0.0, 90.0);
    setPose(goal, 3.0, 2.0, 180.0);
    setPose(goal, 1.0, 2.0, 270.0);
    setPose(goal, 1.0, 0.0, 0.0);

    if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
       ROS_INFO("You have arrived to the goal position");
    }

    else
    {
       ROS_INFO("The base failed for some reason");
    }

    return 0;

}

