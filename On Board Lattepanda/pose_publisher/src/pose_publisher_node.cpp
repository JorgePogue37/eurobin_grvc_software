#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GlobalPose {
public:
    GlobalPose() {
        // Subscribe to /localization
        localization_sub = nh.subscribe("/localization", 10, &GlobalPose::localizationCallback, this);
        // Publishers for both /global_pose and /mavros/vision_pose/pose
        pose_pub_global = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);
        pose_pub_mavros = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber localization_sub;
    ros::Publisher pose_pub_global;
    ros::Publisher pose_pub_mavros;

    void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Extract and rotate orientation from Odometry
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        tf2::Quaternion rotation;
        rotation.setRPY(0, -M_PI / 6, 0);  // Rotate by -30 degrees around the local Y-axis
        quat = quat * rotation;
        quat.normalize();

        // Populate PoseStamped message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;
        pose_msg.pose.orientation = tf2::toMsg(quat);

        // Publish to both topics
        pose_pub_global.publish(pose_msg);
        pose_pub_mavros.publish(pose_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_pose_node");
    GlobalPose node;
    ros::spin();
    return 0;
}

