#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <cmath>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>

#include <thread>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "localization/PoseMsg.h"


// #define M_PI 3.14159265359
#define INITIAL_YAW 240.0

using namespace std;

class Deadreckoning{
    private:
        ros::NodeHandle nh;
        ros::Publisher imu_path_pub;
        ros::Publisher z_calibration_velocity_pub;
        ros::Publisher imu_pose_pub;


        ros::Subscriber gps_velocity_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber utm_coord_sub;

        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point p;           
        // geometry_msgs::Vector3 velocity;
        // geometry_msgs::Vector3 acceleration;
        geometry_msgs::Vector3 calibration_velocity_offsets;
        
        vector<geometry_msgs::Vector3> calibration_velocity_data;
        

        Eigen::Vector3d gravity;
        nav_msgs::Path m_imu_path;
        chrono::steady_clock::time_point start_time;
        ros::Time time;

        tf::TransformBroadcaster tfcaster;
        tf::Transform transform;
        tf::Quaternion q1;
        // std_msgs::Float32 velocity_x;
        // std_msgs::Float32 velocity_y;
        // std_msgs::Float32 velocity_z;
        // std_msgs::Float32 acceleration_x;
        // std_msgs::Float32 acceleration_y;
        // std_msgs::Float32 acceleration_z;
        std_msgs::Float32 calib_velocity_z;

        
        double m_yaw_rate; 
        double m_dVehicleVel_ms;
        double m_prev_velocity;
        bool m_collet_time;

        double m_init_yaw;
        
        double m_utm_x = 0;
        double m_utm_y = 0;
        double m_origin_x = 0;
        double m_origin_y = 0;

        double m_imu_x;
        double m_imu_y;
        double m_imu_yaw;

        double m_velocity_x;
        double m_velocity_y;
        double m_velocity_z;
        
        double m_prev_utm_x = 0;
        double m_prev_utm_y = 0;
        double m_gps_yaw;

        bool initial_time;
        bool m_utm_bool;
        bool m_gps_vel_bool;
        
        bool m_gps_yaw_trigger;

    public :
        double m_delta_time = 0.01;

        Deadreckoning();
        ~Deadreckoning(){};
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_coord_msg);
        void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg);
        void GPSVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
        void IMUDeadReckoning(const geometry_msgs::Vector3 &velocity_msg);
        void Pub();
        double CalcOrientation(const geometry_msgs::Quaternion &msg);
        void CollectCalibrationData(std::vector<geometry_msgs::Vector3>& calibration_data, const geometry_msgs::Vector3& msg);
        void CalibrateSensor(const std::vector<geometry_msgs::Vector3>& calibration_data,
                            geometry_msgs::Vector3& calibration_offsets);
};