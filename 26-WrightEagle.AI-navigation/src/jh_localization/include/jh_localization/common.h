#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// #include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
double  PI = 3.14159265;

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZINormal MapPointType;

template <class class_name>
bool getParameter(const std::string& paramName, class_name& param)
{
    std::string nodeName = ros::this_node::getName();
    std::string paramKey;
    if (!ros::param::search(paramName, paramKey))
    {
        ROS_ERROR("%s: Failed to search for parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    if (!ros::param::has(paramKey))
    {
        ROS_ERROR("%s: Missing required parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    if (!ros::param::get(paramKey, param))
    {
        ROS_ERROR("%s: Failed to get parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    return true;
}
Eigen::Quaterniond ea2quat(double ex, double ey, double ez){
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(ex, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(ey, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(ez, Eigen::Vector3d::UnitZ()));
    
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    return quaternion;
}

double rotation_diff(const Eigen::Matrix3d& diff) {
    double trace = diff.trace();
    
    double angle_radians = std::acos(std::max(std::min((trace - 1) / 2.0, 1.0), -1.0));
    
    if (std::isnan(angle_radians)) {
        return 0.0; // 或者选择其他处理方式
    }

    double angle_degrees = angle_radians * 180.0 / M_PI;

    return angle_degrees;
}