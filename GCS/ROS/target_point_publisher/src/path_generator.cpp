#include "../include/path_generator/path_generator.h"

#include <iostream>
#include <std_msgs/Bool.h>

float current_x;
float current_y;
float current_h;

int endSignal = 0;

std_msgs::Bool goal_received_msg;
std_msgs::Bool success_generating_path_msg;
std_msgs::Bool finished_following_path_msg;

PathGenerator::PathGenerator()
{
    subscribeAndPublish();
}

PathGenerator::~PathGenerator()
{

}

void PathGenerator::subscribeAndPublish(){
    sub_grid_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/path_generator/map", 1, &PathGenerator::gridMapHandler, this);
    sub_nav_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/path_generator/goal/pose", 1, &PathGenerator::navGoalHandler, this); //RVIZ publica en este topico al hacer clic en 2d nav goal
    sub_current_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose", 1, &PathGenerator::updatePosition, this); //recibe posicion
    pub_robot_waypoint_ = nh_.advertise<geometry_msgs::PoseStamped>("/path_generator/waypoints", 1000);
    pub_robot_path_ = nh_.advertise<nav_msgs::Path>("robot_path", 10);
    pub_goal_received_ = nh_.advertise<std_msgs::Bool>("/path_generator/goal/received", 1);
    pub_success_generating_path_ = nh_.advertise<std_msgs::Bool>("/path_generator/success_generating_path", 1);    
    pub_finished_following_path_ = nh_.advertise<std_msgs::Bool>("/path_generator/finished_following_path", 1);
}

void PathGenerator::gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    ROS_INFO("Generating map..");
    map_exsit_ = false;

    map_info_ = map_msg->info;

    // Generate Map, Options
    map_generator_.setWorldSize({(int)map_info_.width, (int)map_info_.height}); //{x, y}
    map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    map_generator_.setDiagonalMovement(true);

    // Add Wall
    int x, y;
    for(int i=0; i<map_info_.width*map_info_.height; i++)
    {
        x = i%map_info_.width;
        y = i/map_info_.width;

        if(map_msg->data[i] != 0)
        {
            map_generator_.addCollision({x, y}, 3);
        }
        if (ros::ok() == false) return;
    }

    ROS_INFO("Success build map!");
    map_exsit_ = true;
}

void PathGenerator::updatePosition(const geometry_msgs::PoseStamped::ConstPtr &current_pose_msg){
    // Round current coordinate
    current_x = round(current_pose_msg->pose.position.x*10)/10;
    current_y = round(current_pose_msg->pose.position.y*10)/10;
    current_h = current_pose_msg->pose.position.z;

}

void PathGenerator::navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
{
    if(!map_exsit_) 
    {
        ROS_INFO("\033[1;31mGoal received but map doesn't exist yet\033[0m");
		pub_goal_received_.publish(goal_received_msg);
		while(!map_exsit_ && endSignal == 0);
    	return;
    }

	double time_tolerance = 1; // Set a tolerance of 0.5 seconds (500 ms)
    // Check if the message timestamp is within the tolerance window
    if ((ros::Time::now() - goal_msg->header.stamp).toSec() > time_tolerance) {
        ROS_INFO("\033[1;31mGoal received but timestamp is too old %f\033[0m", goal_msg->header.stamp.toSec());
		pub_goal_received_.publish(goal_received_msg);
        return; // Ignore the message if it's older than the tolerance window
    }
    
    ROS_INFO("\033[1;32mGoal received!\033[0m");
	pub_goal_received_.publish(goal_received_msg);

    // Round goal coordinate
    float goal_x = round(goal_msg->pose.position.x*10)/10;
    float goal_y = round(goal_msg->pose.position.y*10)/10;
    float goal_z;
    if (goal_msg->pose.position.z != 0) goal_z = goal_msg->pose.position.z;
    else goal_z = current_h;
     
    float initialHeight = current_h;

    float height_step = 0.1;  // Set altitude increment step
    float height_diff = goal_z - initialHeight;
    int num_steps = std::ceil(std::abs(height_diff) / height_step);

    // Fill a vector with altitude steps from initialHeight to goal_z
    std::vector<float> height_values;
    for (int i = 0; i < num_steps; ++i) {
        float height = initialHeight + i * (height_diff / num_steps);
        height_values.push_back(height);
    }
    height_values.push_back(goal_z);  // Ensure exact final height

    // Remmaping coordinate
    AStar::Vec2i target;
    target.x = (goal_x - map_info_.origin.position.x) / map_info_.resolution;
    target.y = (goal_y - map_info_.origin.position.y) / map_info_.resolution;

    AStar::Vec2i source;
    source.x = (current_x - map_info_.origin.position.x) / map_info_.resolution;
    source.y = (current_y - map_info_.origin.position.y) / map_info_.resolution;

    // Find Path
    auto path = map_generator_.findPath(source, target);

    geometry_msgs::PoseStamped path_msg;
    nav_msgs::Path path_rviz;

    if(path.empty())
    {
        ROS_INFO("\033[1;31mFail generate path!\033[0m");
       	success_generating_path_msg.data = false;
		pub_success_generating_path_.publish(success_generating_path_msg);
        return;
    }
    
   	success_generating_path_msg.data = true;
	pub_success_generating_path_.publish(success_generating_path_msg);
	
	int height_index = 0;
	
    // Calculate the distance between current pose and goal pose
    float dx = goal_msg->pose.position.x - current_x;
    float dy = goal_msg->pose.position.y - current_y;
    float distance_to_goal = std::sqrt(dx * dx + dy * dy);

    // Check if distance is within 17 cm (0.17 meters)
    if (distance_to_goal < 0.17) {
        ROS_INFO("Goal is within 17 cm of current position; skipping path generation.");
    }
    else
    {
        for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
        {
            geometry_msgs::PoseStamped point_pose;

            // Remapping coordinate
            point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
            point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
            point_pose.pose.orientation = goal_msg->pose.orientation;
            path_rviz.poses.push_back(point_pose);
            }
            path_rviz.header.frame_id = "map";
            pub_robot_path_.publish(path_rviz);
            ROS_INFO("\033[1;36mSuccess generating path!\033[0m");

        for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
        {
            geometry_msgs::PoseStamped point_pose;
            // Remapping coordinate
            point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
            point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
            point_pose.pose.position.z = height_values[height_index];
            point_pose.pose.orientation = goal_msg->pose.orientation;
            point_pose.header.stamp = ros::Time::now();
            path_msg.pose = point_pose.pose;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = point_pose.header.stamp;
            pub_robot_waypoint_.publish(path_msg);
            if (height_index < height_values.size() - 1) height_index++;
            ros::Duration(0.5).sleep();
        }
    }

    while (height_index < height_values.size())
    {
    	geometry_msgs::PoseStamped point_pose;
        point_pose.pose.position.x = goal_msg->pose.position.x;
        point_pose.pose.position.y = goal_msg->pose.position.y;
        point_pose.pose.position.z = height_values[height_index];
        point_pose.pose.orientation = goal_msg->pose.orientation;
        point_pose.header.stamp = ros::Time::now();
        path_msg.pose = point_pose.pose;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = point_pose.header.stamp;
        pub_robot_waypoint_.publish(path_msg);
        if (height_index < height_values.size()) height_index++;
        ros::Duration(0.5).sleep();
    }

	geometry_msgs::PoseStamped point_pose;
    point_pose.pose.position.x = goal_msg->pose.position.x;
    point_pose.pose.position.y = goal_msg->pose.position.y;
    point_pose.pose.position.z = goal_z;
    point_pose.pose.orientation = goal_msg->pose.orientation;
    point_pose.header.stamp = ros::Time::now();
    path_msg.pose = point_pose.pose;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = point_pose.header.stamp;
    pub_robot_waypoint_.publish(path_msg);


    ROS_INFO("\033[1;34mSuccess following path!\033[0m");
 	pub_finished_following_path_.publish(finished_following_path_msg);   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");

    ROS_INFO("\033[1;32m----> Path Generator Node is Started.\033[0m");
    
    ros::AsyncSpinner spinner(2);
    spinner.start(); 

    PathGenerator PG;

	goal_received_msg.data = true;
	success_generating_path_msg.data = false;
	finished_following_path_msg.data = true;

	while(ros::ok()){
    	ros::spinOnce();
    }
    endSignal = 1;
	spinner.stop();
    return 0;
}
