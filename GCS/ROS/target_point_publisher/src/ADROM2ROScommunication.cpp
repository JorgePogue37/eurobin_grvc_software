#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include "../include/ModuleInterface/ModuleInterface.h"

typedef struct
{
	float pos[3];
	float quat[4];
} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;

using namespace std;

ModuleInterface mi = ModuleInterface(1, "Target_module");

std_msgs::Int32 mode_msg;

void pathgeneratorfinishedfollowingpathCallback(const std_msgs::Bool& msg)
{
	if (msg.data == true)
	{
		int msgCode = 1;
		int msgOption = 0;
		// send signal to ADROM in order to let it know that the positioning with path generator has finished 
		mi.sendMessage(msgCode, msgOption);
	}
}

void arucopositionedCallback(const std_msgs::Bool& msg)
{
	if (msg.data == true)
	{
		int msgCode = 1;
		int msgOption = 0;
		// send signal to ADROM in order to let it know that the positioning with aruco has finished 
		mi.sendMessage(msgCode, msgOption);
	}
}

void movingbackwardsfinishedCallback(const std_msgs::Bool& msg)
{
	if (msg.data == true)
	{
		int msgCode = 1;
		int msgOption = 0;
		// send signal to ADROM in order to let it know that the positioning with aruco has finished 
		mi.sendMessage(msgCode, msgOption);
	}
}

int main(int argc, char** argv) {
	int requestCode = 0;
	int requestOption = 0;
	float pos[3];
	float quat[4];
	
	int errorCode = 0;

	// Create the module interface, indicating module ID and name	
	cout << "Module interface created" << endl;
	
	// Open the UDP interface with the host, indicating host IP, Tx port and Rx port
	mi.openUDPSocket( "127.0.0.1", 25004, 24004);
	cout << "Module interface socket opened" << endl;
	
	// Send a message to the host with code 1, option 2, and robot data
	DATA_PACKET_ROBOT_POSE dataPacketRobotPose = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	int msgCode = 0;
	int msgOption = 0;

    ros::init(argc, argv, "ADROM2ROScommunication_node");
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(2);
    spinner.start(); 
    
    ros::Subscriber aruco_positioned_sub = n.subscribe("/ADROM/aruco/positioned", 1, arucopositionedCallback);
    ros::Subscriber pathgenerator_finished_following_path_sub = n.subscribe("/ADROM/path_generator/positioned", 1, pathgeneratorfinishedfollowingpathCallback);
    ros::Subscriber movingbackwards_finished_sub = n.subscribe("/moving_backwards/finshed", 1, movingbackwardsfinishedCallback);
    
    ros::Publisher	mode_pub = n.advertise<std_msgs::Int32>("/ADROM/mode", 1);
 	ros::Publisher	goal_waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/ADROM/goal/pose", 1);
 	
	while(ros::ok()){
		// Wait for a request
		while(ros::ok() && mi.isRequestReceived() == 0)
		{
			usleep(100000);	// Wait 10 ms
		}
		mi.getRequestParams(requestCode, requestOption, pos, quat);
	    mode_msg.data = requestCode;
	    
	    if (requestCode == 1)
	    {
		    usleep(100000);
			// Create and populate the target pose
			geometry_msgs::PoseStamped target_pose;
			target_pose.header.stamp = ros::Time::now();
			target_pose.header.frame_id = "map"; // or "local_origin" as required
			target_pose.pose.position.x = pos[0];
			target_pose.pose.position.y = pos[1];
			target_pose.pose.position.z = pos[2];

			// Preserve the current orientation of the drone
			target_pose.pose.orientation.x = quat[0];
			target_pose.pose.orientation.y = quat[1];
			target_pose.pose.orientation.z = quat[2];
			target_pose.pose.orientation.w = quat[3];
			
			goal_waypoint_pub.publish(target_pose);
		}
		    
		mode_pub.publish(mode_msg);
		    
        requestCode = 0;
        requestOption = 0;
	}
	spinner.stop();
	mi.closeInterface();
	usleep(100000);
	return 0;
}
