#include <iostream>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>


#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <string>
#include <thread>
#include <cmath>


#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "localization/PoseMsg.h"

#define INITIAL_YAW 0.0
#define N           3  
#define M           3 
using namespace std;



class EKF{
    private:
        ros::NodeHandle nh;
        ros::Subscriber utm_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber gps_vel_sub;

        double m_gps_x_covariance;
        double m_gps_y_covariance;
        double m_velocity_ms;        

     
        double m_utm_x;
        double m_utm_y;

        double m_imu_x;
        double m_imu_y;
        double m_imu_yaw;

        double m_ekf_x;
        double m_ekf_y;
        double m_ekf_yaw;

        double m_init_yaw;

        double m_dt;
        bool init_bool;
        
  

        Eigen::MatrixXd m_P_covariance;
        Eigen::VectorXd m_x_;



    public:
        EKF();
        ~EKF(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg);
        void IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg);
        void ExtendKalmanFileter();
        Eigen::VectorXd EstimatedModel(Eigen::VectorXd& estimated_input);
};

EKF::EKF() : m_P_covariance(N, N), m_x_(N){

    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &EKF::GPSCallback, this);
    gps_vel_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &EKF::GPSCallback, this);
    utm_sub = nh.subscribe("/utm_coord", 1000, &EKF::UTMCallback, this);
    imu_sub = nh.subscribe("/imu_pose", 1000, &EKF::IMUCallback, this);
    init_bool == false;
    nh.param("init_yaw", m_init_yaw, INITIAL_YAW);

};

void EKF::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg){
  
    m_gps_x_covariance = gps_data_msg->position_covariance[0]; //1행 1열 x covariance
    m_gps_y_covariance = gps_data_msg->position_covariance[4]; //2행 2열 y covariance

};
void EKF::GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
    
    // 홀센서 데이터로 수정 예정
    m_velocity_ms = sqrt(pow(gps_velocity_msg->twist.twist.linear.x, 2) + pow(gps_velocity_msg->twist.twist.linear.y, 2));
    
}
void EKF::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg){
    
    m_utm_x = utm_data_msg->x;
    m_utm_y = utm_data_msg->y;
    
};

void EKF::IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg){
    
    m_imu_x = imu_data_msg->pose_x;
    m_imu_y = imu_data_msg->pose_y;
    m_imu_yaw = imu_data_msg->pose_yaw;

};

Eigen::VectorXd EKF::EstimatedModel(Eigen::VectorXd& estimated_input){

    Eigen::VectorXd fk(N);
   
    fk << (estimated_input(0)+ m_velocity_ms * m_dt * cos(estimated_input(2))),
          (estimated_input(1)+ m_velocity_ms * m_dt * sin(estimated_input(2))),
          (estimated_input(2) + m_dt /** md_yawrate*/);

    return fk;

}

void EKF::ExtendKalmanFileter(){
    
    if(!init_bool){

        m_ekf_x = m_utm_x;
        m_ekf_y = m_utm_y;
        m_ekf_yaw = m_init_yaw;

        m_P_covariance << 1000, 0    ,0,
                          0,    1000 ,0,
                          0,    0    ,1000;

         
    }
    else if(init_bool) {
        //추정값과 오차 공분산 예측
        EstimatedModel(m_x_);

    }

}


int main(int argc, char **argv){

    ros::init(argc, argv, "EKF_node");
    EKF ekf;
    ros::Rate loop_rate(100);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}


// [0.025921000000000003, 0.0, 0.0,
// 0.0, 0.025921000000000003, 0.0, 
// 0.0, 0.0, 0.08468099999999999]
