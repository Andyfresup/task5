#include <vector> 
#include <string> 
#include <fstream> 
#include <iostream> 
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>

#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h> 
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h> 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <jh_localization/common.h>
#include <jh_localization/lidarOptimization.h>
#include <jh_localization/tictoc.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

typedef pcl::PointXYZI PointType;

std::queue<sensor_msgs::PointCloud2ConstPtr> currcloudBuf;
ros::Publisher pubtfcloud;
ros::Publisher puborigincloud;
ros::Publisher pubsurfmap;
ros::Publisher pubsegsurfmap;
ros::Publisher publocal_odom;
ros::Publisher pubImuOdometry;


pcl::PointCloud<MapPointType>::Ptr  surface_map (new pcl::PointCloud<MapPointType>);
pcl::PointCloud<MapPointType>::Ptr  seg_surface_map (new pcl::PointCloud<MapPointType>);
pcl::PointCloud<PointType>::Ptr  curr_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  curr_surf_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  tf_cloud (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr  filter_tmp (new pcl::PointCloud<PointType>);
pcl::PointCloud<MapPointType>::Ptr  filter_tmp_normal (new pcl::PointCloud<MapPointType>);


pcl::KdTreeFLANN<MapPointType>::Ptr kdtreeSurfMap;
std::mutex mBuf;
int frame_num = -1;
int N_SCAN = 0;

double lastImuT_imu = -1;

double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; // rx, ry, rz, rw, x, y, z
double update_t[3] = {0, 0, 0};
//  double init_q[4] = {0.045, -0.01, 0.55, 0.82};
// double init_t[3] = {29.9, 34.1, -0.4};
double init_q[4] = {0, 0, 0, 1};
double init_t[3] = {0, 0, 0};

Eigen::Isometry3d odom;
Eigen::Isometry3d last_odom;

Eigen::Isometry3d IMU_odom;
int IMU_odom_nused = 0;

// nav_msgs::Odometry


int optimization_count = 10;
int USE_INIT_ODOM = 1;

Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters+4);



int localization_working = 0;
int init_pose_changed = 0;
double inlier_ratio = 0;

void para_reset(float init_x, float init_y, float init_z, float init_rw, float init_rx, float init_ry, float init_rz){
    parameters[0] = init_rx;
    parameters[1] = init_ry;
    parameters[2] = init_rz;
    parameters[3] = init_rw;
    parameters[4] = init_x;
    parameters[5] = init_y;
    parameters[6] = init_z;
}
void update_init(float init_x, float init_y, float init_z, float init_rw, float init_rx, float init_ry, float init_rz){
    Eigen::Quaterniond inputq(init_rw, init_rx, init_ry, init_rz); // Initial unit quaternion (w, x, y, z)
    ///////////////////////////////
    // Rotate around y-axis 30 degrees
    double angle = -30.0 * M_PI / 180.0; // Convert 30 degrees to radians
    Eigen::AngleAxisd rotation_y(angle, Eigen::Vector3d::UnitX()); // Rotation around Y-axis
    
    // Convert rotation to quaternion
    Eigen::Quaterniond rotation_q(rotation_y);  
    
    // Multiply existing quaternion with new rotation quaternion
    Eigen::Quaterniond result_q = rotation_q * inputq;
    ////////////////////////////////////////////////////////////
    // Eigen::Quaterniond result_q = inputq;  
    ///////////////////////////////////////////////////////////

    init_q[0] = result_q.x();
    init_q[1] = result_q.y();
    init_q[2] = result_q.z();
    init_q[3] = result_q.w();
    init_t[0] = init_x;
    init_t[1] = init_y;
    init_t[2] = init_z;
}
pcl::PointCloud<PointType>::Ptr pointcloudplus(pcl::PointCloud<PointType>::Ptr src1, pcl::PointCloud<PointType>::Ptr src2){
	pcl::PointCloud<PointType>::Ptr outcloud(new pcl::PointCloud<PointType>);
	for(int i = 0;i < src1->points.size();i++)
		outcloud->push_back(src1->points[i]);
	for(int i = 0;i < src2->points.size();i++)
		outcloud->push_back(src2->points[i]);
	return outcloud;
}
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudIn)
{
    mBuf.lock();
    currcloudBuf.push(cloudIn);
    mBuf.unlock();
}


void initHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    init_q[0] = msg->pose.pose.orientation.x;
    init_q[1] = msg->pose.pose.orientation.y;
    init_q[2] = msg->pose.pose.orientation.z;
    init_q[3] = msg->pose.pose.orientation.w;
    init_t[0] = msg->pose.pose.position.x;
    init_t[1] = msg->pose.pose.position.y;
    init_t[2] = msg->pose.pose.position.z;
    // init_t[2] = 8;

    init_pose_changed = 1;
    localization_working = 0;
    ROS_ERROR("reset location! x: %f, y: %f", init_t[0], init_t[1]);
}


pcl::PointCloud<PointType>::Ptr  TransformCloud(pcl::PointCloud<PointType>::Ptr cloudin,    Eigen::Quaterniond q,  Eigen::Vector3d  t)
{
    pcl::PointCloud<PointType>::Ptr cloudout(new pcl::PointCloud<PointType> );
    PointType tmppoint;
    for(size_t i = 0;i < cloudin->points.size(); i++){
        Eigen::Vector3d point(cloudin->points[i].x, cloudin->points[i].y, cloudin->points[i].z);
        Eigen::Vector3d un_point =q * point +t;
        
        tmppoint.x = un_point.x();
        tmppoint.y = un_point.y();
        tmppoint.z = un_point.z();
        tmppoint.intensity = cloudin->points[i].intensity;
        cloudout->points.push_back(tmppoint);
    }
    return cloudout;
}


std::mutex mtx; // Global mutex lock
std::vector<Eigen::Vector3d> curr_point_list;
std::vector<Eigen::Vector3d> norm_list;
std::vector<double> negative_OA_dot_norm_list;
std::vector<double> weight_list;

void add_to_vectors(Eigen::Vector3d curr_point, Eigen::Vector3d norm, double negative_OA_dot_norm, double weight = 1.0) {
    std::lock_guard<std::mutex> lock(mtx); // Lock the mutex
    curr_point_list.push_back(curr_point);
    norm_list.push_back(norm);
    negative_OA_dot_norm_list.push_back(negative_OA_dot_norm);
    weight_list.push_back(weight);
}




void pointAssociateToMap(PointType const *const pi, MapPointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}



pcl::PointCloud<PointType>::Ptr feature_extract(pcl::PointCloud<PointType>::Ptr cloudIn){
    pcl::PointCloud<PointType>::Ptr surf(new pcl::PointCloud<PointType>);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(cloudIn);

    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudIn->points.size(); i++){
        std::vector<int> search_indices; //point index Vector
        std::vector<float> distances;	//distance Vector
        // std::vector<int>().swap(search_indices);
        // std::vector<float>().swap(distances);
        
        tree.radiusSearch(cloudIn->points[i], 1, search_indices, distances); 
        // get_pca_feature(cloudIn, search_indices, features[i]);

        int pt_num = search_indices.size();
        if (search_indices.size() <= 3)
            continue;

        pcl::PointCloud<PointType>::Ptr selected_cloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < search_indices.size(); ++i)
            selected_cloud->points.push_back(cloudIn->points[search_indices[i]]);

        pcl::PCA<PointType> pca_operator;
        pca_operator.setInputCloud(selected_cloud);

        // Compute eigen values and eigen vectors
        Eigen::Vector3f eigen_values = pca_operator.getEigenValues();
        
        float lamada1 = eigen_values(0);
        float lamada2 = eigen_values(1);
        float lamada3 = eigen_values(2);
        float planar_2 = (lamada2 - lamada3) / lamada1;

        if(planar_2 > 0.7)
            #pragma omp critical
            surf->points.push_back(cloudIn->points[i]);
    }
    return surf;
}


pcl::PointCloud<PointType>::Ptr feature_extract2(pcl::PointCloud<PointType>::Ptr cloudIn, float sample_ratio = 0.5f) {
    pcl::PointCloud<PointType>::Ptr surf(new pcl::PointCloud<PointType>);
    
    pcl::RandomSample<PointType> random_sample;
    random_sample.setInputCloud(cloudIn);
    
    int sample_size = static_cast<int>(cloudIn->size() * sample_ratio);
    random_sample.setSample(sample_size);
    
    random_sample.setSeed(2025);
    random_sample.filter(*filter_tmp);
    *surf = *filter_tmp;

    return surf;
}


sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in){
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    // acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    // gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from ;//* extQRPY
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    // if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    // {
    //     ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
    //     ros::shutdown();
    // }

    return imu_out;
}


void calculate_odometry(const pcl::PointCloud<MapPointType>::Ptr& surf_map, const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud){

    int opti_counter = 2;
    getParameter("opti_counter", opti_counter);
    if(optimization_count>opti_counter)
        optimization_count--;
    if(USE_INIT_ODOM){
        Eigen::Isometry3d init_odom = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond tmp_q(init_q[3], init_q[0], init_q[1], init_q[2]);
        Eigen::Matrix3d rotation_matrix =  tmp_q.matrix();
        Eigen::Vector3d tmp_t(init_t[0], init_t[1], init_t[2]);
        init_odom.rotate(rotation_matrix);
        init_odom.pretranslate(tmp_t);
        odom = init_odom;
        last_odom = odom;
        USE_INIT_ODOM = 0;
    }
    else{
        if(IMU_odom_nused){
            Eigen::Isometry3d odom_diff = odom.inverse() * IMU_odom;
            double  diff_angles = rotation_diff(odom_diff.rotation());
            ROS_INFO("diff_angles: %f", diff_angles);

            if(diff_angles > 20){
                last_odom = odom;
                odom = IMU_odom;
                IMU_odom_nused = 0;
                ROS_ERROR("USE IMU ODOM!!!!!!!!!!!!!!!!!!!!!");

            }
            else{
                Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
                last_odom = odom;
                odom = odom_prediction;
            }
            
        }
        else{
            Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
            last_odom = odom;
            odom = odom_prediction;
        }  
    }    

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    int surf_num=0;

    if(surf_map->points.size()>50){
        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            // addSurfCostFactor;
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            
            TicToc t_surface;
            #pragma omp parallel for num_threads(8)
            for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
            {
                MapPointType point_temp;
                pointAssociateToMap(&(downsampledSurfCloud->points[i]), &point_temp);
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {
                    
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = surf_map->points[pointSearchInd[j]].x;
                        matA0(j, 1) = surf_map->points[pointSearchInd[j]].y;
                        matA0(j, 2) = surf_map->points[pointSearchInd[j]].z;
                    }

                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * surf_map->points[pointSearchInd[j]].x +
                                norm(1) * surf_map->points[pointSearchInd[j]].y +
                                norm(2) * surf_map->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }

                    Eigen::Vector3d curr_point(downsampledSurfCloud->points[i].x, downsampledSurfCloud->points[i].y, downsampledSurfCloud->points[i].z);
                    if (planeValid)
                        add_to_vectors(curr_point, norm, negative_OA_dot_norm);
                }

            }
            surf_num = curr_point_list.size();

            if(surf_num<20){
                printf("not enough surf points\n");
            }
            
            for (int i = 0; i < (int)curr_point_list.size(); i++){
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point_list[i], norm_list[i], negative_OA_dot_norm_list[i]);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);
            }

            curr_point_list.clear();
            norm_list.clear();
            negative_OA_dot_norm_list.clear();
            weight_list.clear();
            std::cout<<"t_surface:"<<t_surface.toc()<<std::endl;
                    
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////

            TicToc t_solve;

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            options.num_threads = std::thread::hardware_concurrency();

            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            std::cout<<"t_solve:"<<t_solve.toc()<<std::endl;
            
        }
    }else{
        printf("not enough points in map to associate, map error");
    }
    inlier_ratio = double(surf_num) / double(downsampledSurfCloud->points.size());
    std::cout<<"ratio:                                      "<<inlier_ratio<<std::endl;
    q_w_curr.normalize();
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;


    if( inlier_ratio < 0.1){ //40km/h, == 11.1 m/s
        localization_working = 0;
        ROS_ERROR("Localization failed, reset parameters");
        // update_init(init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz);
    }

  
}

void calculate_odometry2(const pcl::PointCloud<MapPointType>::Ptr& surf_map, const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud){

    int opti_counter = 2;
    getParameter("opti_counter", opti_counter);
    if(optimization_count>opti_counter)
        optimization_count--;
    if(USE_INIT_ODOM){
        Eigen::Isometry3d init_odom = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond tmp_q(init_q[3], init_q[0], init_q[1], init_q[2]);
        Eigen::Matrix3d rotation_matrix =  tmp_q.matrix();
        Eigen::Vector3d tmp_t(init_t[0], init_t[1], init_t[2]);
        init_odom.rotate(rotation_matrix);
        init_odom.pretranslate(tmp_t);
        odom = init_odom;
        last_odom = odom;
        USE_INIT_ODOM = 0;
    }
    else{
        if(IMU_odom_nused){
            Eigen::Isometry3d odom_diff = odom.inverse() * IMU_odom;
            double  diff_angles = rotation_diff(odom_diff.rotation());
            ROS_INFO("diff_angles: %f", diff_angles);

            if(diff_angles > 20){
                last_odom = odom;
                odom = IMU_odom;
                IMU_odom_nused = 0;
                ROS_ERROR("USE IMU ODOM!!!!!!!!!!!!!!!!!!!!!");

            }
            else{
                Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
                last_odom = odom;
                odom = odom_prediction;
            }
            
        }
        else{
            Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
            last_odom = odom;
            odom = odom_prediction;
        }  
    }    

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    int surf_num=0;

    if(surf_map->points.size()>50){
        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            // addSurfCostFactor;
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            
            TicToc t_surface;
            #pragma omp parallel for num_threads(8)
            for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
            {
                MapPointType point_temp;
                pointAssociateToMap(&(downsampledSurfCloud->points[i]), &point_temp);
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeSurfMap->nearestKSearch(point_temp, 1, pointSearchInd, pointSearchSqDis);
                if(pointSearchSqDis[0] > 0.5)
                    continue;
                auto &pt = surf_map->points[pointSearchInd[0]];
                Eigen::Vector3d norm(pt.normal_x, pt.normal_y, pt.normal_z);
                norm.normalize();
                double d = - (norm(0) * pt.x + norm(1) * pt.y + norm(2) * pt.z);
                double dist = fabs(norm(0) * pt.x + norm(1) * pt.y + norm(2) * pt.z + d);
                if (dist > 0.2)
                    continue;
               
                Eigen::Vector3d curr_point(downsampledSurfCloud->points[i].x, downsampledSurfCloud->points[i].y, downsampledSurfCloud->points[i].z);
                add_to_vectors(curr_point, norm, d);

            }
            surf_num = curr_point_list.size();

            if(surf_num<20){
                printf("not enough surf points\n");
            }
            std::cout<<"surf_num:"<<surf_num<<std::endl;
            for (int i = 0; i < (int)curr_point_list.size(); i++){
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point_list[i], norm_list[i], negative_OA_dot_norm_list[i]);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);
            }

            curr_point_list.clear();
            norm_list.clear();
            negative_OA_dot_norm_list.clear();
            weight_list.clear();
            std::cout<<"t_surface:"<<t_surface.toc()<<std::endl;
                    
            //////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////

            TicToc t_solve;

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            // options.num_threads = 32;
          
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            std::cout<<"t_solve:"<<t_solve.toc()<<std::endl;
            
        }
    }else{
        printf("not enough points in map to associate, map error");
    }
    inlier_ratio = double(surf_num) / double(downsampledSurfCloud->points.size());
    std::cout<<"ratio:                                      "<<inlier_ratio<<std::endl;
    q_w_curr.normalize();
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    if(inlier_ratio < 0.1){ //40km/h, == 11.1 m/s
        localization_working = 0;
        ROS_ERROR("Localization failed, reset parameters");
        // update_init(init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz);
    }

}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "sonic_localization"); 
    ros::NodeHandle nh; 
    std::string ws_path;
    std::string topic = "/jh_cloud";
    std::string imu_topic;
    double feature_size;
    double ds_ratio = 0.5;
    nh.getParam("/ws_path", ws_path);
    std::cout<<"Loading map"<<std::endl;
    pcl::io::loadPCDFile<MapPointType>(ws_path+"/data/map/surfaceMap.pcd", *surface_map);
    std::cout<<"Finish Loading map"<<std::endl;

    // nh.getParam("/topic", topic);
    nh.getParam("/imu_topic", imu_topic);
    nh.getParam("/feature_size", feature_size);
    nh.getParam("/ds_ratio", ds_ratio);



    ros::Subscriber subcloud = nh.subscribe<sensor_msgs::PointCloud2>(topic, 100, cloudHandler);

    ros::Subscriber subinitialpose = nh.subscribe("/initialpose", 1000, initHandler);//rviz
    pubtfcloud = nh.advertise<sensor_msgs::PointCloud2> ("tf_cloud", 1);
    puborigincloud = nh.advertise<sensor_msgs::PointCloud2> ("origin_cloud", 1);
    pubsegsurfmap = nh.advertise<sensor_msgs::PointCloud2> ("seg_surface_map", 1);
    pubsurfmap = nh.advertise<sensor_msgs::PointCloud2> ("surface_map", 1);
    publocal_odom = nh.advertise<nav_msgs::Odometry>("/local_odom", 100);
    pubImuOdometry   = nh.advertise<nav_msgs::Odometry>("/odom", 2000);


    sensor_msgs::PointCloud2 cloudtempmsg;
    
    // ros::Duration(1).sleep(); // Delay for debugging
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surface_map, *surface_map, indices);


    
 
    pcl::VoxelGrid<MapPointType> downSizeFilterRAW;

    downSizeFilterRAW.setLeafSize(feature_size*2, feature_size*2, feature_size*2);
    downSizeFilterRAW.setInputCloud(surface_map);
    downSizeFilterRAW.filter(*filter_tmp_normal);
    *surface_map = *filter_tmp_normal;
    
    float imuAccNoise, imuGyrNoise, imuAccBiasN, imuGyrBiasN, imuGravity;
    getParameter("imuAccNoise", imuAccNoise);
    getParameter("imuGyrNoise", imuGyrNoise);
    getParameter("imuAccBiasN", imuAccBiasN);
    getParameter("imuGyrBiasN", imuGyrBiasN);
    getParameter("imuGravity", imuGravity);




    
    float init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz;
    getParameter("/translation/x", init_x);
    getParameter("/translation/y", init_y);
    getParameter("/translation/z", init_z);
    getParameter("/rotation/x", init_rx);
    getParameter("/rotation/y", init_ry);
    getParameter("/rotation/z", init_rz);
    getParameter("/rotation/w", init_rw);
    update_init(init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz);

    

    seg_surface_map->clear();
    pcl::PassThrough<MapPointType> pass;
    pass.setInputCloud (surface_map);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (init_t[0] - 202, init_t[0] + 202);
    pass.setNegative (false);
    pass.filter (*seg_surface_map);
    pass.setInputCloud (seg_surface_map);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (init_t[1] - 202, init_t[1] + 202);
    pass.setNegative (false);
    pass.filter (*filter_tmp_normal);
    *seg_surface_map = *filter_tmp_normal;


    kdtreeSurfMap = pcl::KdTreeFLANN<MapPointType>::Ptr(new pcl::KdTreeFLANN<MapPointType>());
    kdtreeSurfMap->setInputCloud(seg_surface_map);
    
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(feature_size, feature_size, feature_size);

    // std::thread offer_map_process{offer_map};
    std::cout<<"------------start localization-----------------"<<std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
        // ros::Duration(0.1).sleep(); // Delay for debugging
        
        if (!currcloudBuf.empty()){
            //std::cout<<"start>>>>>>>"<<std::endl;
            frame_num += 1;
            mBuf.lock();
            while (currcloudBuf.size() > 1) {
                currcloudBuf.pop();
            }
            ros::Time timestamp;
            if (!currcloudBuf.empty()) {
                timestamp = currcloudBuf.back()->header.stamp;
                pcl::fromROSMsg(*currcloudBuf.back(), *curr_cloud);
                currcloudBuf.pop();  // Pop and remove from queue
            }
            mBuf.unlock();

            if(init_pose_changed){
                para_reset(init_t[0],init_t[1],init_t[2], init_q[3], init_q[0],init_q[1],init_q[2]);

                pcl::PassThrough<MapPointType> pass;
                pass.setInputCloud (surface_map);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (parameters[4] - 202, parameters[4] + 202);
                pass.setNegative (false);
                pass.filter (*seg_surface_map);
                pass.setInputCloud (seg_surface_map);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (parameters[5] - 202, parameters[5] + 202);
                pass.setNegative (false);
                pass.filter (*filter_tmp_normal);
                *seg_surface_map = *filter_tmp_normal;
                
                update_t[0] = parameters[4];
                update_t[1] = parameters[5];
                update_t[2] = parameters[6];
                kdtreeSurfMap->setInputCloud(seg_surface_map);
                
                para_reset(init_t[0],init_t[1],init_t[2], init_q[3], init_q[0],init_q[1],init_q[2]);

                // update_init(init_x, init_y, init_z, init_rw, init_rx, init_ry, init_rz);
                init_pose_changed = 0;
                localization_working = 1;
                USE_INIT_ODOM = 1;
            }
            if(localization_working == 0)
                continue;
            

            TicToc  t_whole;
            TicToc t_feature;
            std::cout<<"frame cloud size:"<< curr_cloud->points.size()<<std::endl;
            downSizeFilterSurf.setInputCloud(curr_cloud);
            downSizeFilterSurf.filter(*filter_tmp); 
            *curr_cloud = *filter_tmp;
            curr_surf_cloud->clear();
            // curr_surf_cloud = feature_extract(curr_cloud, in);
            curr_surf_cloud = feature_extract2(curr_cloud, ds_ratio);
            std::cout<<"feature extraction time: "<<t_feature.toc()<<std::endl;
            std::cout<<"feature size: "<<curr_surf_cloud->points.size()<<std::endl;

          


            
            double distance_for_update_map = (parameters[4] - update_t[0])*(parameters[4] - update_t[0]) + (parameters[5] - update_t[1])*(parameters[5] - update_t[1]) 
                                            + (parameters[6] - update_t[2]) * (parameters[6] - update_t[2]);
            // std::cout<<"#########"<<parameters[4]<<" "<<update_t[0]<<" "<<parameters[5]<<" "<<update_t[1]<<" "<<parameters[6]<<" "<<update_t[2]<<std::endl;
            if (distance_for_update_map > 25){
                pcl::PassThrough<MapPointType> pass;
                pass.setInputCloud (surface_map);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (parameters[4] - 202, parameters[4] + 202);
                pass.setNegative (false);
                pass.filter (*seg_surface_map);
                pass.setInputCloud (seg_surface_map);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (parameters[5] - 202, parameters[5] + 202);
                pass.setNegative (false);
                pass.filter (*filter_tmp_normal);
                *seg_surface_map = *filter_tmp_normal;

                update_t[0] = parameters[4];
                update_t[1] = parameters[5];
                update_t[2] = parameters[6];
                kdtreeSurfMap->setInputCloud(seg_surface_map);
            }

            TicToc t_odom;
            std::cout<<"map size: "<<seg_surface_map->points.size()<<std::endl;
            calculate_odometry2(seg_surface_map, curr_surf_cloud);
            std::cout<<"t_odom:"<<t_odom.toc()<<std::endl;

            update_init(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], parameters[6]);
            std::cout<<"t: "<<parameters[4]<<" "<<parameters[5]<<" "<<parameters[6]<<std::endl;
            std::cout<<"q: "<<parameters[0]<<" "<< parameters[1]<<" "<< parameters[2]<<" "<<parameters[3]<<std::endl;

            Eigen::Quaterniond q(parameters[3], parameters[0], parameters[1], parameters[2]);
            Eigen::Vector3d  t(parameters[4], parameters[5], parameters[6]);
            TicToc t_tfcloud;
            tf_cloud = TransformCloud(curr_cloud , q,t);
            // std::cout<<"t_tfcloud:"<<t_tfcloud.toc()<<std::endl;

            // static tf::TransformBroadcaster br;
            // tf::Transform transform;
            // transform.setOrigin( tf::Vector3(t.x(), t.y(), t.z()) );
            // tf::Quaternion q_tf(q.x(),q.y(),q.z(),q.w());
            // transform.setRotation(q_tf);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            

            std::cout<<"all time:"<<t_whole.toc()<<std::endl<<std::endl;

            pcl::toROSMsg(*curr_cloud, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            puborigincloud.publish(cloudtempmsg);

             pcl::toROSMsg(*surface_map, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubsurfmap.publish(cloudtempmsg);

            pcl::toROSMsg(*tf_cloud, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubtfcloud.publish(cloudtempmsg);

            pcl::toROSMsg(*seg_surface_map, cloudtempmsg);
            cloudtempmsg.header.frame_id = "map";
            pubsegsurfmap.publish(cloudtempmsg);

            nav_msgs::Odometry local_odom;
            local_odom.header.stamp = timestamp;
            local_odom.pose.pose.position.x = parameters[4];
            local_odom.pose.pose.position.y = parameters[5];
            local_odom.pose.pose.position.z = parameters[6];
            local_odom.pose.pose.orientation.x = parameters[0];
            local_odom.pose.pose.orientation.y = parameters[1];
            local_odom.pose.pose.orientation.z = parameters[2];
            local_odom.pose.pose.orientation.w = parameters[3];
            local_odom.header.frame_id = "map";
            publocal_odom.publish(local_odom);
        }  
    }
    return 0; 
}
