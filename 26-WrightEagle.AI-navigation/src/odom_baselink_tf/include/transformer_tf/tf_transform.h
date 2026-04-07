#ifndef TF_TRANSFORM_H
#define TF_TRANSFORM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class transformer_tf
{
    public:
        transformer_tf();
        ~transformer_tf();
        void init();
        void run();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        std::string odom_frame_;
        std::string map_frame_;
        std::string base_foot_frame_;
	std::string rslidar_frame_;
        ros::Subscriber sub_odom_;

        void OdomTFMapCallback(const nav_msgs::Odometry& odom_msg);
};

#endif
