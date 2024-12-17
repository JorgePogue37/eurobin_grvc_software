// Libraries
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <thread>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Bool.h>

using namespace std;

#define UDP_PORT_MULTIRROTOR_ARUCO 28000

// Structure definition
typedef struct
{
	char header[3];		// "RPP" (Robot Pose Packet) character sequence
    uint8_t updated;
	float pos[3];		// Position of the Aruco marker
	float rot[3];		// Rotation angles of Aruco marker
} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;


// Global variables
int endThreadSignal = 0;
int validArucoPose = 0;
float arucoMarkerPosition[3] = {0.0, 0.0, 0.0};
float arucoMarkerRotation[3] = {0.0, 0.0, 0.0};

float comandedDronePosition[3];

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped last_waypoint;

ros::Publisher target_pub;  // Declare publisher globally

std_msgs::Bool success_msg;

tf2::Vector3 aruco_deviation;

float arucoDesiredPosition[3];
tf2::Vector3 cameraCoordinatesFromLidar;

float xMotionRange;
float yMotionRange;
float zMotionRange;

float maxIncrease;
float axis_alignment_tolerance;
float tolerance_for_reached_point;
float distance_error_to_desired_position = FLT_MAX;
// Thread function declaration
void receiveArucoMRPoseThread();

// Callback to get the current pose of the drone
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    last_waypoint = *msg;
    comandedDronePosition[0] = last_waypoint.pose.position.x;
    comandedDronePosition[1] = last_waypoint.pose.position.y;
    comandedDronePosition[2] = last_waypoint.pose.position.z;    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_guidance_aruco_node");
    ros::NodeHandle n;

    thread arucoMarkerThread;

    // Subscriber to obtain the current orientation of the drone
    ros::Subscriber pose_sub = n.subscribe("/global_pose", 1, poseCallback);
    ros::Subscriber last_waypoint_sub = n.subscribe("/aruco_guidance/reference_waypoint", 1, waypointCallback);

	ros::Publisher pub_point_reached = n.advertise<std_msgs::Bool>("/aruco_guidance/positioned", 1);

    // Publisher to send the target position
    target_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco_guidance/vision/pose", 1);

    // Initialize the global position in case is not received
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = 1;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = 0;
    current_pose.pose.orientation.w = 1;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "map";        

    // Initialize the last waypoint in case is not received
    last_waypoint.pose.position.x = 0.0;
    last_waypoint.pose.position.y = 0.0;
    last_waypoint.pose.position.z = 0.1;
    last_waypoint.pose.orientation.x = 0;
    last_waypoint.pose.orientation.y = 0;
    last_waypoint.pose.orientation.z = 0;
    last_waypoint.pose.orientation.w = 1;
    last_waypoint.header.stamp = ros::Time::now();
    last_waypoint.header.frame_id = "map";
    
    arucoDesiredPosition[0] = 0.629;
    arucoDesiredPosition[1] = 0.043;
    arucoDesiredPosition[2] = -0.285;
    
    // Inicialization of the marker position on the desired position so no deviation exists until the first point has arrived
    arucoMarkerPosition[0] = arucoDesiredPosition[0];
    arucoMarkerPosition[1] = arucoDesiredPosition[1];
    arucoMarkerPosition[2] = arucoDesiredPosition[2];

    xMotionRange = 0.6;
    yMotionRange = 0.6;
    zMotionRange = 0.4;

    maxIncrease = 0.07;
    
    axis_alignment_tolerance = 0.1;    //if aruco_deviation in Z axis is bigger than this value, the new position coamnded to the drone just changes in this axis, if it's smaller but it's bigger on the Y axis it only affects to these two axis, and if Z and Y axis are smaller it affects to every axis
    
    tolerance_for_reached_point = 0.05;

	// Create the thread for receiving Aruco marker pose through UDP socket
	arucoMarkerThread = thread(&receiveArucoMRPoseThread);
    arucoMarkerThread.detach();

    ros::Rate loop_rate(20);

    while (ros::ok() && endThreadSignal == 0)
    {   
        // Create and populate the target pose
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = "map"; // or "local_origin" as required
        target_pose.pose.position.x = comandedDronePosition[0];
        target_pose.pose.position.y = comandedDronePosition[1];
        target_pose.pose.position.z = comandedDronePosition[2];

        // Preserve the current orientation of the drone
        target_pose.pose.orientation = last_waypoint.pose.orientation;
        
		if (distance_error_to_desired_position < tolerance_for_reached_point)
		{	
			success_msg.data = true;
			pub_point_reached.publish(success_msg);
		}
        
        // Publish the target position with orientation
        target_pub.publish(target_pose);
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    endThreadSignal = 1;
    usleep(100000);

    return 0;
}

/*
 * Thread for receiving Aruco Data
 */
void receiveArucoMRPoseThread()
{
	DATA_PACKET_ROBOT_POSE * dataPacketRobotPose;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int k = 0;
    double theta = 0.349;
	int errorCode = 0;


	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		errorCode = 1;
		cout << endl << "ERROR: could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(UDP_PORT_MULTIRROTOR_ARUCO);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		errorCode = 1; 
		cout << endl << "ERROR: could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(errorCode == 0 && endThreadSignal == 0)
	{	
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if (dataReceived > 0)
		{
			// Check if message is correct
			if(buffer[0] == 'R' && buffer[1] == 'P' && buffer[2] == 'P')
			{	
				// Get the code sent by the GCS
				dataPacketRobotPose = (DATA_PACKET_ROBOT_POSE*)buffer;
				
                if(dataPacketRobotPose->updated == 1){
                    // Extract the fields
                    arucoMarkerPosition[0] = cos(theta)*dataPacketRobotPose->pos[0] + sin(theta)*dataPacketRobotPose->pos[2];
                    arucoMarkerPosition[1] = dataPacketRobotPose->pos[1];
                    arucoMarkerPosition[2] = -sin(theta)*dataPacketRobotPose->pos[0] + cos(theta)*dataPacketRobotPose->pos[2];
                    for(k = 0; k < 3; k++)
                        arucoMarkerRotation[k] = dataPacketRobotPose->rot[k];
                    //cout << "Posición recibida de Aruco:   " << arucoMarkerPosition[0] << "   " << arucoMarkerPosition[1] << "    " << arucoMarkerPosition[2] << endl;

                    tf2::Transform transform;
                    tf2::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
                    transform.setRotation(q);

					aruco_deviation[0] = arucoMarkerPosition[0] - arucoDesiredPosition[0];
					aruco_deviation[1] = arucoMarkerPosition[1] - arucoDesiredPosition[1];
					aruco_deviation[2] = arucoMarkerPosition[2] - arucoDesiredPosition[2]; 
					
					distance_error_to_desired_position = aruco_deviation.length();

					//cout << aruco_deviation[0] << "   "  << aruco_deviation[1] << "     " << aruco_deviation[2] << endl;

                    float increase = aruco_deviation.length();
                    if (increase > maxIncrease) increase = maxIncrease;
                    
                    if (abs(aruco_deviation[2]) > axis_alignment_tolerance)
                    {
                        aruco_deviation[0] = 0;
                        aruco_deviation[1] = 0;
                        aruco_deviation = aruco_deviation.normalize() * increase;
                    } else if (abs(aruco_deviation[1]) > axis_alignment_tolerance)
                    {
                        aruco_deviation[0] = 0;
                        aruco_deviation = aruco_deviation.normalize() * increase;
                    } else aruco_deviation = aruco_deviation.normalize() * increase;
                    
                    //cout << aruco_deviation[0] << "   "  << aruco_deviation[1] << "     " << aruco_deviation[2] << endl; 
                    //cout << aruco_deviation.length() << endl;
                    aruco_deviation = transform * aruco_deviation;

                    comandedDronePosition[0] = current_pose.pose.position.x + aruco_deviation[0];
                    comandedDronePosition[1] = current_pose.pose.position.y + aruco_deviation[1];
                    comandedDronePosition[2] = current_pose.pose.position.z + aruco_deviation[2];

                    // Limit the commanded position within the specified range from the last waypoint
                    comandedDronePosition[0] = std::max(static_cast<float>(last_waypoint.pose.position.x - xMotionRange/2), std::min(static_cast<float>(last_waypoint.pose.position.x + xMotionRange/2), comandedDronePosition[0]));
                    comandedDronePosition[1] = std::max(static_cast<float>(last_waypoint.pose.position.y - yMotionRange/2), std::min(static_cast<float>(last_waypoint.pose.position.y + yMotionRange/2), comandedDronePosition[1]));
                    comandedDronePosition[2] = std::max(static_cast<float>(last_waypoint.pose.position.z - zMotionRange/2), std::min(static_cast<float>(last_waypoint.pose.position.z + zMotionRange/2), comandedDronePosition[2]));
                }
            }
		}
		else
		{
			// Wait 2 ms
			usleep(10000);
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);	
}
