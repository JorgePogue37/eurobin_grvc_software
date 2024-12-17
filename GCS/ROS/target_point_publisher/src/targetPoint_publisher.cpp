// Libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <mavros_msgs/State.h>

int mode = 0; // 0 for waiting mode, 1 for pathGenerator, 2 for Aruco
int previous_mode = 0;

int step_back = 0;

bool endsignal = false;

bool new_goal_received = false;
bool success_generating_path = false;
bool finished_following_path = false;

float time_tolerance = 1;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped goal_path_generator_pose;
geometry_msgs::PoseStamped comanded_drone_pose;
geometry_msgs::PoseStamped last_drone_comanded_pose;

geometry_msgs::PoseStamped pathgenerator_pose;
geometry_msgs::PoseStamped aruco_pose;
geometry_msgs::PoseStamped movingbackwards_pose;

mavros_msgs::State Drone_State;

ros::Publisher	aruco_reference_waypoint_pub;
ros::Publisher	moving_backwards_finished_pub;
ros::Publisher	ADROM_goalreached_pub;
ros::Publisher	ADROM_arucopositioned_pub;

void currentposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void ADROMposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	goal_path_generator_pose = *msg;
	if (Drone_State.mode == "GUIDED")
	{
		aruco_reference_waypoint_pub.publish(goal_path_generator_pose);
	}
	if (mode == 1 && Drone_State.mode == "GUIDED")
	{	
		ros::Publisher goal_path_generator_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/path_generator/goal/pose", 1);
		new_goal_received = false;
		success_generating_path = false;
		finished_following_path = false;
		tf2::Vector3 distance_back;
		distance_back[0] = 0.05;
		distance_back[1] = 0;
		distance_back[2] = 0;
	 	int i = 0;
		geometry_msgs::PoseStamped aux_goal_path_generator_pose = goal_path_generator_pose;
		tf2::Transform transform;
		tf2::Quaternion q(goal_path_generator_pose.pose.orientation.x, goal_path_generator_pose.pose.orientation.y, goal_path_generator_pose.pose.orientation.z, goal_path_generator_pose.pose.orientation.w);
		transform.setRotation(q);
		
		while(success_generating_path != true && endsignal != true && mode == 1)
		{
			while(new_goal_received != true && endsignal !=true && mode == 1) 
			{
				goal_path_generator_pub.publish(aux_goal_path_generator_pose);
				ros::Duration(0.1).sleep();
			}
			if(success_generating_path == false) {
				new_goal_received = false;
				i++;
				aux_goal_path_generator_pose.pose.position.x = goal_path_generator_pose.pose.position.x - (transform * (i*distance_back)).x();
				aux_goal_path_generator_pose.pose.position.y = goal_path_generator_pose.pose.position.y - (transform * (i*distance_back)).y();
				aux_goal_path_generator_pose.pose.position.z = goal_path_generator_pose.pose.position.z - (transform * (i*distance_back)).z();
				aux_goal_path_generator_pose.header.stamp = ros::Time::now();
			}
		}
		while (finished_following_path != true && endsignal !=true  && mode == 1);
		for( i; i>=0; i--) 
		{
			pathgenerator_pose.pose.position.x = goal_path_generator_pose.pose.position.x - (transform * (i*distance_back)).x();
			pathgenerator_pose.pose.position.y = goal_path_generator_pose.pose.position.y - (transform * (i*distance_back)).y();
			pathgenerator_pose.pose.position.z = goal_path_generator_pose.pose.position.z - (transform * (i*distance_back)).z();
			pathgenerator_pose.header.stamp = ros::Time::now();
		    ros::Duration(0.5).sleep();
		}
		pathgenerator_pose = goal_path_generator_pose;
		pathgenerator_pose.header.stamp = ros::Time::now();
		ros::Duration(0.1).sleep();
		
		new_goal_received = false;
		success_generating_path = false;
		finished_following_path = false;	
		mode = 0;
		
		std_msgs::Bool success_msg;
		success_msg.data = true;
		ADROM_goalreached_pub.publish(success_msg);
	}
}

void modeCallback(const std_msgs::Int32& msg)
{
    mode = msg.data;
}

void PathGeneratorposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pathgenerator_pose = *msg;
}

void ARUCOposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_pose = *msg;
}

void arucopositionedCallback(const std_msgs::Bool& msg)
{
	if (msg.data == true && mode == 2)
	{
		std::cout << "positioned with aruco" << std::endl;
		std_msgs::Bool success_msg;
		success_msg.data = true;
		ADROM_arucopositioned_pub.publish(success_msg);
		mode = 0;
	}
}

void pathgeneratorgoalreceivedCallback(const std_msgs::Bool& msg)
{
	new_goal_received = msg.data;
}

void pathgeneratorsuccessgeneratingpathCallback(const std_msgs::Bool& msg)
{
	success_generating_path = msg.data;
}


void pathgeneratorfinishedfollowingpathCallback(const std_msgs::Bool& msg)
{
	finished_following_path = msg.data;
}

void DroneModeCallback(const mavros_msgs::State::ConstPtr& msg)
{
	Drone_State = *msg;
}

void movingBackwards()
{
	tf2::Transform transform;
	tf2::Quaternion q(goal_path_generator_pose.pose.orientation.x, goal_path_generator_pose.pose.orientation.y, goal_path_generator_pose.pose.orientation.z, goal_path_generator_pose.pose.orientation.w);
	transform.setRotation(q);
	tf2::Vector3 distance_back;
	distance_back[0] = 0.01;
	distance_back[1] = 0;
	distance_back[2] = 0; 
		
	movingbackwards_pose.pose.position.x = goal_path_generator_pose.pose.position.x - (transform * (step_back*distance_back)).x();
	movingbackwards_pose.pose.position.y = goal_path_generator_pose.pose.position.y - (transform * (step_back*distance_back)).y();
	movingbackwards_pose.pose.position.z = goal_path_generator_pose.pose.position.z - (transform * (step_back*distance_back)).z();
	movingbackwards_pose.pose.orientation = goal_path_generator_pose.pose.orientation;
	movingbackwards_pose.header.stamp = ros::Time::now();

	step_back++;
	if (step_back >= 90)
	{
		mode = 0;
		step_back = 0;	
		std_msgs::Bool success_msg;
		success_msg.data = true;
		moving_backwards_finished_pub.publish(success_msg);
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_comanding_node");
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(2);
    spinner.start(); 

    ros::Subscriber current_pose_sub = n.subscribe("/global_pose", 1, currentposeCallback);
    ros::Subscriber ADROM_pose_sub = n.subscribe("/ADROM/goal/pose", 1, ADROMposeCallback);
    ros::Subscriber ADROM_mode_sub = n.subscribe("/ADROM/mode", 1, modeCallback);
        
    ros::Subscriber pathgenerator_pose_sub = n.subscribe("/path_generator/waypoints", 1, PathGeneratorposeCallback);
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_guidance/vision/pose", 1, ARUCOposeCallback);
    
    ros::Subscriber aruco_positioned_sub = n.subscribe("/aruco_guidance/positioned", 1, arucopositionedCallback);
    
    ros::Subscriber drone_mode_sub = n.subscribe("/mavros/state", 1, DroneModeCallback);
    
    ros::Subscriber pathgenerator_goal_received_sub = n.subscribe("/path_generator/goal/received", 1, pathgeneratorgoalreceivedCallback);
    ros::Subscriber pathgenerator_success_generating_path_sub = n.subscribe("/path_generator/success_generating_path", 1, pathgeneratorsuccessgeneratingpathCallback);
    ros::Subscriber pathgenerator_finished_following_path_sub = n.subscribe("/path_generator/finished_following_path", 1, pathgeneratorfinishedfollowingpathCallback);

	aruco_reference_waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco_guidance/reference_waypoint", 1);
	moving_backwards_finished_pub = n.advertise<std_msgs::Bool>("/moving_backwards/finshed", 1);
    ADROM_goalreached_pub = n.advertise<std_msgs::Bool>("/ADROM/path_generator/positioned", 1);
    ADROM_arucopositioned_pub = n.advertise<std_msgs::Bool>("/ADROM/aruco/positioned", 1);
    
    ros::Publisher	comanded_drone_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

    ros::Rate loop_rate(10);
    
    std::cout << "Starting in waiting mode" << std::endl;
    
    Drone_State.mode = "GUIDED";

	while(ros::ok()){
		
		if (mode == 0 || (mode == 1 && success_generating_path == false))			//Waiting mode
		{
			comanded_drone_pose = last_drone_comanded_pose;
		}
		
		if (mode == 1 && success_generating_path == true)							//ADROM path generator
		{
			if ((ros::Time::now() - pathgenerator_pose.header.stamp).toSec() < time_tolerance)
			{
				comanded_drone_pose = pathgenerator_pose;
				last_drone_comanded_pose = comanded_drone_pose;
			} else comanded_drone_pose = last_drone_comanded_pose;
		}
		
		if (mode == 2)																//ARUCO guidance
		{
			if ((ros::Time::now() - aruco_pose.header.stamp).toSec() < time_tolerance)
			{
				comanded_drone_pose = aruco_pose;
				last_drone_comanded_pose = comanded_drone_pose;
			} else comanded_drone_pose = last_drone_comanded_pose;
		}
		
		if (mode == 3)																//box taken, moving backwards
		{
			movingBackwards();
			comanded_drone_pose = movingbackwards_pose;
			last_drone_comanded_pose = comanded_drone_pose;
		} else step_back = 0;
		
		if (Drone_State.mode != "GUIDED")
		{
			comanded_drone_pose = current_pose;
			last_drone_comanded_pose = comanded_drone_pose;
			goal_path_generator_pose = last_drone_comanded_pose;
			aruco_reference_waypoint_pub.publish(last_drone_comanded_pose);
			mode = 0;
		}
		
		if (mode != previous_mode)
		{
			if (mode != 0 && mode != 1 && mode != 2 && mode != 3)
			{
				mode = 0;
				std::cout << "mode received does not exist" << std::endl;
			}
			if (mode == 0) std::cout << "Changing to waiting mode" << std::endl;
			if (mode == 1) std::cout << "Changing to path generator mode" << std::endl;
			if (mode == 2) std::cout << "Changing to aruco mode" << std::endl;
			if (mode == 3) std::cout << "Changing to moving backwards mode" << std::endl;
		}
		previous_mode = mode;
		
		comanded_drone_pose_pub.publish(comanded_drone_pose);		//publish comanded drone pose in mavros
		
        loop_rate.sleep();
		ros::spinOnce;		
	}
	endsignal = true;
	spinner.stop();
	return 0;
}
