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

#define INITIAL_YAW 81.3
#define N           3  
#define M           2 
using namespace std;



class EKF{
    private:
        ros::NodeHandle nh;
        ros::Subscriber utm_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber gps_vel_sub;

        ros::Publisher ekf_path_pub;

        double m_gps_x_covariance;
        double m_gps_y_covariance;
        double m_velocity_ms;        

        double m_utm_x;
        double m_utm_y;
        double m_utm_yaw;
        double m_prev_utm_x;
        double m_prev_utm_y;

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

        visualization_msgs::Marker ekf_path;
        geometry_msgs::Point p;       
        tf::TransformBroadcaster tfcaster;
        tf::Transform transform;
        tf::Quaternion q1;

        Eigen::MatrixXd m_P_post, m_P_prior, A_jacb, H_jacb, Q_noise_cov, R_noise_cov, K_gain;
        Eigen::VectorXd m_x_post, m_x_prior, m_z_measured;

    public:
        EKF();
        ~EKF(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg);
        void IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg);
        void ExtendKalmanFilter();
        void EKFPathVisualize();
        void Pub();

        Eigen::VectorXd EstimatedModel(Eigen::VectorXd& estimated_input);
        Eigen::VectorXd MeasurementModel(Eigen::VectorXd& measured_input);
};

EKF::EKF() : m_P_post(N, N), m_P_prior(N, N), A_jacb(N, N), H_jacb(M, N), Q_noise_cov(N, N), K_gain(N, M), R_noise_cov(M, M),
             m_x_post(N), m_x_prior(N), m_z_measured(M){

    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &EKF::GPSCallback, this);
    gps_vel_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &EKF::GPSVelCallback, this);
    utm_sub = nh.subscribe("/utm_coord", 1000, &EKF::UTMCallback, this);
    imu_sub = nh.subscribe("/imu_pose", 1000, &EKF::IMUCallback, this);

    ekf_path_pub = nh.advertise<visualization_msgs::Marker>("/ekf_path", 1000);

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
    // cout << "----------------" << endl;
    // cout << "X covariance of gps : " << m_gps_x_covariance << endl;
    // cout << "Y covariance of gps : " << m_gps_y_covariance << endl;
    gps_sub_bool = true;

};

void EKF::GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
    
    // 홀센서 데이터로 수정 예정
    m_velocity_ms = sqrt(pow(gps_velocity_msg->twist.twist.linear.x, 2) + pow(gps_velocity_msg->twist.twist.linear.y, 2));
    // cout << "----------------" << endl;
    // cout << m_velocity_ms << endl;
    gps_velocity_sub_bool = true;
};

void EKF::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg){
    
    m_utm_x = utm_data_msg->x;
    m_utm_y = utm_data_msg->y;
    // cout << "----------------" << endl;
    // cout << m_utm_x << ", " << m_utm_y << endl;
    utm_sub_bool = true;
};

void EKF::IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg){
    
    m_imu_x = imu_data_msg->pose_x;
    m_imu_y = imu_data_msg->pose_y;
    m_imu_yaw = imu_data_msg->pose_yaw;
    m_yaw_rate = imu_data_msg->yaw_rate;
    // cout << "----------------" << endl;
    // cout << m_yaw_rate << endl;
    imu_sub_bool = true;

};

Eigen::VectorXd EKF::EstimatedModel(Eigen::VectorXd& estimated_input){

    Eigen::VectorXd fk(N);
   
    fk << (estimated_input(0)+ m_velocity_ms * m_dt * cos(estimated_input(2))),
          (estimated_input(1)+ m_velocity_ms * m_dt * sin(estimated_input(2))),
          (estimated_input(2) + m_dt * m_yaw_rate);

    return fk;

};

Eigen::VectorXd EKF::MeasurementModel(Eigen::VectorXd& measured_input){

    Eigen::VectorXd hk(M);
    
    //gps의 yaw는 없다고 가정하여 계산
    hk << (measured_input(0)),
          (measured_input(1));
    
    return hk;

};

void EKF::ExtendKalmanFilter(){

    if(!init_bool && utm_sub_bool){
       
        // m_x_prior << m_utm_x,
        //              m_utm_y;
        //             //  m_init_yaw;
        m_x_post(0) = m_utm_x;
        m_x_post(1) = m_utm_y;
        m_x_post(2) = m_init_yaw;
        cout << "----------------" << endl;
        cout << "X prior : " << m_x_post(0) << endl;
        cout << "Y prior : " << m_x_post(1) << endl;
        cout << "Yaw prior : " << m_x_post(2) << endl;
  

        m_P_post << 1000, 0,    0,
                    0,    1000, 0,
                    0,    0,    1000;
        
    
        init_bool = true;
         
    }
    if(init_bool){
        
        //예측 시스템 모델의 자코비안(시스템 모델을 각 변수로 미분한 행렬)
        A_jacb << 1, 0, (- 1 * m_velocity_ms * m_dt * sin(m_x_post(2))),
                  0, 1, (1 * m_velocity_ms * m_dt * cos(m_x_post(2))),
                  0, 0, 1;
        // cout << "-------A_jacb-------" <<endl;
        // cout << A_jacb << endl;
        //측정 시스템 모델의 자코비안
        H_jacb << 1, 0, 0,
                  0, 1, 0;
        // cout << "-------H_jacb-------" <<endl;
        // cout << H_jacb << endl;
         // prediction 공분산 예시
        Q_noise_cov <<  0.01, 0.0,     0.0,             
                        0.0,     0.01, 0.0,              // 센서 오차 감안 (휴리스틱)
                        0.0,     0.0,     0.01;          // 직접 센서를 움직여본 후 공분산 구할 것
        // cout << "-------Q_noise_cov-------" <<endl;
        // cout << Q_noise_cov << endl;
        // measerment 공분산
        R_noise_cov << m_gps_x_covariance, 0,                 
                       0,                  m_gps_y_covariance;
                     // gps yaw의 covariance는 어떻게? -> gps yaw를 고려안한다면 오차 공분산 = X
        // cout << "-------R_noise_cov-------" <<endl;
        // cout << R_noise_cov << endl;
        // measurement value
        m_z_measured << m_utm_x,
                        m_utm_y;    // gps yaw는 어떻게?
        //  cout << "-------m_z_measured-------" <<endl;
        // cout << m_z_measured << endl;

        if(imu_sub_bool){
            
            // cout << "Debugging1" << endl;
            //1. 추정값과 오차 공분산 예측
            m_x_prior = EstimatedModel(m_x_post);
            // cout << "-------m_x_prior-------" << endl;
            // cout << m_x_prior << endl;

            m_P_prior = A_jacb * m_P_post * A_jacb.transpose() + Q_noise_cov;
            // cout << "-------m_P_prior-------" << endl;
            // cout << m_P_prior << endl;

            // cout << "Debugging2" << endl;
            //2. 칼만 이득 계산
            K_gain = m_P_prior.inverse() * H_jacb.transpose() * (H_jacb * m_P_prior * H_jacb.transpose() + R_noise_cov).inverse();
            // cout << "-------K_gain-------" << endl;
            // cout << K_gain << endl;
            
            // cout << "Debugging3" << endl;
            //3. 추정값 계산
            m_x_post = m_x_prior + K_gain * (m_z_measured - MeasurementModel(m_x_prior));
            // cout << "Debugging4" << endl;
            //4. 오차 공분산 계산
            m_P_post = m_P_prior - K_gain * H_jacb * m_P_prior.inverse();
        }
        

    }

}

void EKF::Pub(){
    cout << "-----------" << endl;
    cout << "EKF X : " << m_x_post(0) << endl;
    cout << "EKF Y : " << m_x_post(1) << endl;
    cout << "EKF Yaw : " << m_x_post(2) << endl;
    EKFPathVisualize();
}

void EKF::EKFPathVisualize(){
    ekf_path.header.frame_id = "world"; // Set the frame id
    ekf_path.header.stamp = ros::Time::now();
    ekf_path.ns = "line_strip";
    ekf_path.action = visualization_msgs::Marker::ADD;
    ekf_path.type = visualization_msgs::Marker::LINE_STRIP;
    ekf_path.pose.orientation.x = 0;
    ekf_path.pose.orientation.y = 0;
    ekf_path.pose.orientation.z = 0;
    ekf_path.pose.orientation.w = 1.0;
    ekf_path.scale.x = 0.3; // Line width

    // Set the marker color (RGBA)
    ekf_path.color.r = 0.2;
    ekf_path.color.g = 0.8;
    ekf_path.color.b = 0.0;
    ekf_path.color.a = 1.0;
    p.x = m_x_post(0);
    p.y = m_x_post(1);
    p.z = 0;

    transform.setOrigin(tf::Vector3(p.x, p.y, 0.0));
    q1.setRPY(0, 0, 0);
    transform.setRotation(q1);
    tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ekf_frame"));

    ekf_path.points.push_back(p);
    ekf_path_pub.publish(ekf_path);

};


int main(int argc, char **argv){

    ros::init(argc, argv, "EKF_node");
    EKF ekf;
    ros::Rate loop_rate(100);

    while(ros::ok()){
        ekf.ExtendKalmanFilter();
        ekf.Pub();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}


// [0.025921000000000003, 0.0, 0.0,
// 0.0, 0.025921000000000003, 0.0, 
// 0.0, 0.0, 0.08468099999999999]
