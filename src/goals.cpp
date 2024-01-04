#include <iostream>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

int main(int argc, char  **argv){
    ros::init(argc, argv, "goals");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);


    // arbitrary goal points
    vector<vector<float>> points{ {3.0,0}  , {3,3} , {0,3}};

    for (int i=0; i<points.size(); i++)
    {
        MoveBaseClient ac("move_base", true);
         //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
          ROS_INFO("Waiting for the move_base action server to come up");
        }
   
        move_base_msgs::MoveBaseGoal goal;
   
        //we'll send a goal point to the robot 
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
   
        goal.target_pose.pose.position.x = points[i][0];
        goal.target_pose.pose.position.y = points[i][1];
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;
   
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
   
        ac.waitForResult();
   
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("Hooray, the base reached the goal point");
        else
          ROS_INFO("The base failed to move toward the goal point");

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
