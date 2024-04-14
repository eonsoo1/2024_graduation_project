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
        double m_yaw_rate;

        double m_ekf_x;
        double m_ekf_y;
        double m_ekf_yaw;

        double m_init_yaw;

        double m_dt = 0.01;
        bool init_bool;
        bool gps_sub_bool;
        bool gps_velocity_sub_bool;
        bool utm_sub_bool;
        bool imu_sub_bool;
            
  

        Eigen::MatrixXd m_P_post, m_P_prior, A_jacb, H_jacb, Q_noise_cov, R_noise_cov, K_gain;
        Eigen::VectorXd m_x_post, m_x_prior, m_z_measured;



    public:
        EKF();
        ~EKF(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg);
        void IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg);
        void ExtendKalmanFileter();
        Eigen::VectorXd EstimatedModel(Eigen::VectorXd& estimated_input);
        Eigen::VectorXd MeasurementModel(Eigen::VectorXd& measured_input);
};

EKF::EKF() : m_P_post(N, N), m_P_prior(N, N), A_jacb(N, N), H_jacb(M, M), Q_noise_cov(N, N), K_gain(N, N), R_noise_cov(M, M),
             m_x_post(N), m_x_prior(N), m_z_measured(M){

    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &EKF::GPSCallback, this);
    gps_vel_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &EKF::GPSVelCallback, this);
    utm_sub = nh.subscribe("/utm_coord", 1000, &EKF::UTMCallback, this);
    imu_sub = nh.subscribe("/imu_pose", 1000, &EKF::IMUCallback, this);
    init_bool = false;
    gps_sub_bool = false;
    gps_velocity_sub_bool = false;
    utm_sub_bool = false;
    imu_sub_bool = false;
    nh.param("init_yaw", m_init_yaw, INITIAL_YAW);

};

void EKF::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg){
  
    m_gps_x_covariance = gps_data_msg->position_covariance[0]; //1행 1열 x covariance
    m_gps_y_covariance = gps_data_msg->position_covariance[4]; //2행 2열 y covariance

    gps_sub_bool = true;

};
void EKF::GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
    
    // 홀센서 데이터로 수정 예정
    m_velocity_ms = sqrt(pow(gps_velocity_msg->twist.twist.linear.x, 2) + pow(gps_velocity_msg->twist.twist.linear.y, 2));
    
    gps_velocity_sub_bool = true;
}
void EKF::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg){
    
    m_utm_x = utm_data_msg->x;
    m_utm_y = utm_data_msg->y;
    
    utm_sub_bool = true;
};

void EKF::IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg){
    
    m_imu_x = imu_data_msg->pose_x;
    m_imu_y = imu_data_msg->pose_y;
    m_imu_yaw = imu_data_msg->pose_yaw;
    m_yaw_rate = imu_data_msg->yaw_rate;

    imu_sub_bool = true;

};

Eigen::VectorXd EKF::EstimatedModel(Eigen::VectorXd& estimated_input){

    Eigen::VectorXd fk(N);
   
    fk << (estimated_input(0)+ m_velocity_ms * m_dt * cos(estimated_input(2))),
          (estimated_input(1)+ m_velocity_ms * m_dt * sin(estimated_input(2))),
          (estimated_input(2) + m_dt * m_yaw_rate);

    return fk;

}

Eigen::VectorXd EKF::MeasurementModel(Eigen::VectorXd& measured_input){

    Eigen::VectorXd hk(N);
   
    hk << (measured_input(0)),
          (measured_input(1)),
          (measured_input(2));

    return hk;

}

void EKF::ExtendKalmanFileter(){
    
    if(!init_bool){

        m_x_prior(0) = m_utm_x;
        m_x_prior(1) = m_utm_y;
        m_x_prior(2) = m_init_yaw;

        m_P_post << 1000, 0    ,0,
                          0,    1000 ,0,
                          0,    0    ,1000;

        init_bool = true;
         
    }
    else{
        
        //예측 시스템 모델의 자코비안(시스템 모델을 각 변수로 미분한 행렬)
        A_jacb << 1, 0, - m_x_post(2) * m_velocity_ms * m_dt * sin(m_x_post(2)),
                      0, 1, m_x_post(2) * m_velocity_ms * m_dt * cos(m_x_post(2)),
                      0, 0, 1;
        
        //측정 시스템 모델의 자코비안
        H_jacb << 1, 0, 0,
                  0, 1, 0,
                  0, 0, 1;
        
         // prediction 공분산 예시
        Q_noise_cov <<  0.00001, 0.0,     0.0,             
                        0.0,     0.00001, 0.0,              // 센서 오차 감안 (휴리스틱)
                        0.0,     0.0,     0.00001;          // 직접 센서를 움직여본 후 공분산 구할 것
        
        // measerment 공분산
        R_noise_cov << m_gps_x_covariance, 0, 0,
                       0,                  m_gps_y_covariance, 0,
                       0,                  0,                  0.00001; // gps yaw의 covariance는 어떻게?

        // measurement value
        m_z_measured << m_utm_x, 0, 0,
                        0,       m_utm_y, 0,
                        0,       0,       0;    // gps yaw는 어떻게?
                        
        if(imu_sub_bool){
            
            //1. 추정값과 오차 공분산 예측
            m_x_prior = EstimatedModel(m_x_prior);
            m_P_prior = A_jacb * m_P_post * A_jacb.transpose() + Q_noise_cov;
            
            //2. 칼만 이득 계산
            K_gain = m_P_prior.inverse() * H_jacb.transpose() * (H_jacb * m_P_prior * H_jacb.transpose() + R_noise_cov).inverse();

            //3. 추정값 계산
            m_x_post = m_x_prior + K_gain * (m_z_measured - MeasurementModel(m_x_prior));

            //4. 오차 공분산 계산
            m_P_post = m_P_prior - K_gain * H_jacb * m_P_prior.inverse();
        }
        

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
