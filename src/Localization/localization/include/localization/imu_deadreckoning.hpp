#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <thread>

#define PI 3.14159265359

using namespace std;

class Deadreckoning{
    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        ros::Publisher z_calibration_velocity_pub;
        ros::Subscriber imu_sub;

        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point p;           
        // geometry_msgs::Vector3 velocity;
        // geometry_msgs::Vector3 acceleration;
        geometry_msgs::Vector3 calibration_velocity_offsets;
        
        vector<geometry_msgs::Vector3> calibration_velocity_data;
        
        Eigen::Vector3d gravity;
        visualization_msgs::Marker imu_path;
        chrono::steady_clock::time_point start_time;
        ros::Time time;

        // std_msgs::Float32 velocity_x;
        // std_msgs::Float32 velocity_y;
        // std_msgs::Float32 velocity_z;
        // std_msgs::Float32 acceleration_x;
        // std_msgs::Float32 acceleration_y;
        // std_msgs::Float32 acceleration_z;
        std_msgs::Float32 calib_velocity_z;

        double m_delta_time;
        double m_yaw_rate; 
        double m_dVehicleVel_ms;
        double m_prev_velocity;
        bool m_collet_time;
     
        double m_imu_x;
        double m_imu_y;
        double m_imu_yaw;
        bool initial_time = true;

    public :
        Deadreckoning();
        ~Deadreckoning(){};
        void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg);
        void IMUDeadReckoning(const geometry_msgs::Vector3 &velocity_msg, 
                                    const geometry_msgs::Vector3 &accel_msg);
        void Pub();
        void CalcOrientation(const geometry_msgs::Quaternion &msg);
        void CollectCalibrationData(std::vector<geometry_msgs::Vector3>& calibration_data, const geometry_msgs::Vector3& msg);
        void CalibrateAccelerometer(const std::vector<geometry_msgs::Vector3>& calibration_data,
                            geometry_msgs::Vector3& calibration_offsets);
};