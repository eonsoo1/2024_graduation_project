#include <iostream>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

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

struct Pose{
    double x;
    double y;
    double yaw;
};


class EKF{
    private:
        ros::NodeHandle nh;
        ros::Subscriber utm_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber gps_vel_sub;
        ros::Publisher utm_path_pub;
        ros::Publisher imu_path_pub;
        ros::Publisher ekf_path_pub;
        ros::Publisher ekf_pose_pub;
        ros::Publisher imu_pose_pub;
        ros::Publisher utm_pose_pub;

        double m_gps_x_covariance;
        double m_gps_y_covariance;
        double m_velocity_ms;        
        double m_prev_utm_x;
        double m_prev_utm_y;
        double m_yaw_rate;

        Pose m_utm;
        Pose m_imu;
        Pose m_ekf;
        Pose m_init;
        
        

        bool m_init_bool;
        bool m_utm_bool;
        bool m_utm_update_bool;
        bool m_gps_sub_bool;
        bool m_gps_velocity_sub_bool;
        bool m_utm_yaw_trigger;
        bool m_utm_sub_bool;
        bool m_imu_sub_bool;

        nav_msgs::Path m_utm_path;
        nav_msgs::Path m_imu_path;
        nav_msgs::Path m_ekf_path;
        nav_msgs::Odometry m_ekf_odom;
        nav_msgs::Odometry m_imu_odom;
        nav_msgs::Odometry m_utm_odom;
        
        geometry_msgs::Point p_e;
        geometry_msgs::Point p_i;       
        geometry_msgs::Point p_u;

        tf::TransformBroadcaster tfcaster_ekf;
        tf::Transform transform_ekf;
        tf::Quaternion q_e;
        tf::TransformBroadcaster tfcaster_imu;
        tf::Transform transform_imu;
        tf::Quaternion q_i;
        tf::TransformBroadcaster tfcaster_utm;
        tf::Transform transform_utm;
        tf::Quaternion q_u;


        Eigen::MatrixXd m_P_post, m_P_prior, A_jacb, H_jacb, Q_noise_cov, R_noise_cov, K_gain;
        Eigen::VectorXd m_imu_dr, m_x_post, m_x_prior, m_x_dr, m_z_measured;

    public:
        EKF();
        ~EKF(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg);
        void IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg);
        void ExtendKalmanFilter();
        void EKFPathVisualize();
        void IMUPathVisualize();
        void UTMPathVisualize();
        
        void Pub();

        Eigen::VectorXd EstimatedModel(Eigen::VectorXd& estimated_input);
        Eigen::VectorXd MeasurementModel(Eigen::VectorXd& measured_input);
        double m_dt = 0.0125;
};

EKF::EKF() : m_P_post(N, N), m_P_prior(N, N), A_jacb(N, N), H_jacb(M, N), Q_noise_cov(N, N), K_gain(N, M), R_noise_cov(M, M),
             m_imu_dr(N), m_x_post(N), m_x_prior(N), m_z_measured(M), m_x_dr(N){

    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &EKF::GPSCallback, this);
    gps_vel_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &EKF::GPSVelCallback, this);
    utm_sub = nh.subscribe("/utm_coord", 1000, &EKF::UTMCallback, this);
    imu_sub = nh.subscribe("/imu_pose", 1000, &EKF::IMUCallback, this);
    
    ekf_path_pub = nh.advertise<nav_msgs::Path>("/EKF_path", 1000);
    imu_path_pub = nh.advertise<nav_msgs::Path>("/IMU_path", 1000);
    utm_path_pub = nh.advertise<nav_msgs::Path>("/UTM_path", 1000);

    ekf_pose_pub = nh.advertise<nav_msgs::Odometry>("/EKF_odom", 1000);
    imu_pose_pub = nh.advertise<nav_msgs::Odometry>("/IMU_odom", 1000);
    utm_pose_pub = nh.advertise<nav_msgs::Odometry>("/UTM_odom", 1000);

    
    m_init_bool = false;
    m_gps_sub_bool = false;
    m_gps_velocity_sub_bool = false;
    m_utm_yaw_trigger = false;
    m_utm_sub_bool = false;
    m_imu_sub_bool = false;
    m_utm_update_bool = false;
    m_utm_bool = false;
    m_prev_utm_x = 0;
    m_prev_utm_y = 0;

    nh.param("init_yaw", m_init.yaw, INITIAL_YAW);

};

void EKF::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg){
  
    m_gps_x_covariance = gps_data_msg->position_covariance[0]; //1행 1열 x covariance (m 단위)
    m_gps_y_covariance = gps_data_msg->position_covariance[4]; //2행 2열 y covariance (m 단위)
   
    m_gps_sub_bool = true;

};

void EKF::GPSVelCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
    
    // 홀센서 데이터로 수정 예정
    m_velocity_ms = sqrt(pow(gps_velocity_msg->twist.twist.linear.x, 2) + pow(gps_velocity_msg->twist.twist.linear.y, 2)); // (m/s 단위)
    m_gps_velocity_sub_bool = true;
};

void EKF::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg){
    
    if(!m_utm_bool){
        m_utm.x = utm_data_msg->x;
        m_utm.y = utm_data_msg->y;
        
        m_init.x = m_utm.x;
        m_init.y = m_utm.y;
        m_prev_utm_x = m_utm.x;
        m_prev_utm_y = m_utm.y;

        m_utm_bool =true;
    }
    else{
        m_utm.x = utm_data_msg->x;
        m_utm.y = utm_data_msg->y;

        if(m_utm.x != m_prev_utm_x){
            m_utm_update_bool = true;
            m_utm.yaw = atan2((m_utm.y - m_prev_utm_y), (m_utm.x - m_prev_utm_x));
        }
            else{
            m_utm_update_bool = false;
        }
        double distance;
        distance = sqrt(pow((m_utm.x - m_init.x), 2) + pow((m_utm.y - m_init.y), 2));
        if(m_velocity_ms > 0.1 && distance > 3.0 && !m_utm_yaw_trigger){//0.1
            m_init.yaw = atan2((m_utm.y - m_init.y), (m_utm.x - m_init.x));
            m_utm_yaw_trigger = true;
        }

        m_prev_utm_x = m_utm.x;
        m_prev_utm_y = m_utm.y;
    }
    

};

void EKF::IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg){
    

    m_yaw_rate = imu_data_msg->yaw_rate;
    m_imu_sub_bool = true;

};

Eigen::VectorXd EKF::EstimatedModel(Eigen::VectorXd& estimated_input){

    Eigen::VectorXd fk(N);
    Eigen::VectorXd p_before(2), p_after(2);
    Eigen::MatrixXd R(2,2);
    
    fk << (estimated_input(0) + m_velocity_ms * m_dt * cos(estimated_input(2))),
          (estimated_input(1) + m_velocity_ms * m_dt * sin(estimated_input(2))),
          (estimated_input(2) + m_dt * m_yaw_rate);


    return fk;

};

Eigen::VectorXd EKF::MeasurementModel(Eigen::VectorXd& measured_input){

    Eigen::VectorXd hk(M);
    
    //gps의 yaw를 고려하지 않고 계산
    hk << (measured_input(0)),
          (measured_input(1));
          
    //gps의 yaw를 고려하고 계산
    // hk << (measured_input(0)),
    //       (measured_input(1)),
    //       (measured_input(2));
        
    return hk;

};

void EKF::ExtendKalmanFilter(){
    if(m_utm_yaw_trigger){
   
        if(!m_init_bool){
        
            m_x_post(0) = m_utm.x;
            m_x_post(1) = m_utm.y;
            m_x_post(2) = m_init.yaw;

            m_imu_dr(0) = m_utm.x;
            m_imu_dr(1) = m_utm.y;
            m_imu_dr(2) = m_init.yaw;
            // cout << "----------------" << endl;
            // cout << "X prior : " << m_x_post(0) << endl;
            // cout << "Y prior : " << m_x_post(1) << endl;
            // cout << "Yaw prior : " << m_x_post(2) << endl;
            m_P_post << 1000, 0,    0,
                        0,    1000, 0,
                        0,    0,    1000;
            
        
            m_init_bool = true;
            
        }
        else if(m_init_bool){
            
            //예측 시스템 모델의 자코비안(시스템 모델을 각 변수로 미분한 행렬)
            A_jacb << 1, 0, (- 1 * m_velocity_ms * m_dt * sin(m_x_post(2))),
                    0, 1, (1 * m_velocity_ms * m_dt * cos(m_x_post(2))),
                    0, 0, 1;
            //측정 시스템 모델의 자코비안
            H_jacb << 1, 0, 0,
                      0, 1, 0;

            //gps yaw를 고려한 자코비안
            // H_jacb << 1, 0, 0,
            //         0, 1, 0,
            //         0, 0, 1;

            double alpha = 1e-10;

            // prediction 공분산 예시
            Q_noise_cov <<  alpha * m_gps_x_covariance,  0.0,     0.0,             
                            0.0,    alpha * m_gps_y_covariance, 0.0,              // 센서 오차 감안 (휴리스틱)
                            0.0,     0.0,     0.000000005;          // 직접 센서를 움직여본 후 공분산 구할 것
            
            
            // gps yaw를 고려하지 않은 measerment 공분산
            R_noise_cov << m_gps_x_covariance,  0,                
                           0,                   m_gps_y_covariance;
        
            // gps yaw를 고려하지 않은 measurement value
            m_z_measured << m_utm.x,
                            m_utm.y;
   

            // gps yaw를 고려한 measerment 공분산               
            // R_noise_cov << m_gps_x_covariance,  0,                   0,
            //                0,                   m_gps_y_covariance, 0,
            //                0,                   0,                  1;
            
            //gps yaw를 고려한 measerment value
            // m_z_measured << m_utm.x,
            //                 m_utm.y,
            //                 m_utm.yaw; 

            // if(m_imu_sub_bool){

                //1. 추정값과 오차 공분산 예측
                m_x_prior = EstimatedModel(m_x_post);
                m_P_prior = A_jacb * m_P_post * A_jacb.transpose() + Q_noise_cov;


                m_imu_dr = EstimatedModel(m_imu_dr);
                

                cout << "-----------Q noise----------- \n" <<
                         Q_noise_cov << endl;
                cout << "-----------R noise----------- \n" <<
                         R_noise_cov << endl;
                // measure 값이 들어왔을 때 ekf 실행
                m_x_post = m_x_prior;
                m_P_post = m_P_prior;

                
                

                if(m_utm_update_bool){                 
                    //2. 칼만 이득 계산
                    K_gain = m_P_prior * H_jacb.transpose() * (H_jacb * m_P_prior * H_jacb.transpose() + R_noise_cov).inverse();
                    //3. 추정값 계산
                    m_x_post = m_x_prior + K_gain * (m_z_measured - MeasurementModel(m_x_prior));
                    //4. 오차 공분산 계산
                    m_P_post = m_P_prior - K_gain * H_jacb * m_P_prior;


                }
         
            // }
            

        }
    }
}

void EKF::Pub(){

    cout << "----------------" << endl;
    cout << "GPS(utm) update bool : " << m_utm_update_bool << endl;
    

    cout << "-----------" << endl;
    cout << "GPS(utm) X : " << m_z_measured(0) << endl;
    cout << "GPS(utm) Y : " << m_z_measured(1) << endl;
    // cout << "GPS(utm) Yaw : " << m_z_measured(2) * 180 / M_PI << endl;
    
    cout << "-----------" << endl;
    cout << "IMU DR X : " << m_x_prior(0) << endl;
    cout << "IMU DR Y : " << m_x_prior(1) << endl;
    cout << "IMU DR Yaw : " << m_x_prior(2) * 180 / M_PI << endl;
    cout << "-----------" << endl;
    cout << "EKF X : " << m_x_post(0) << endl;
    cout << "EKF Y : " << m_x_post(1) << endl;
    cout << "EKF Yaw : " << m_x_post(2) * 180 / M_PI << endl;
   
    EKFPathVisualize();
    IMUPathVisualize();
    UTMPathVisualize();
    
}

void EKF::EKFPathVisualize(){

    // Path Line visualiazation
    geometry_msgs::PoseStamped ekf_pose;
    
    m_ekf_odom.header.frame_id = "world"; // Set the frame id
    m_ekf_odom.header.stamp = ros::Time::now();

    m_ekf_path.header.frame_id = "world"; // Set the frame id
    m_ekf_path.header.stamp = ros::Time::now();

    p_e.x = m_x_post(0);
    p_e.y = m_x_post(1);
    p_e.z = 0;
    
    ekf_pose.pose.position = p_e;
    ekf_pose.pose.orientation.w = 1;

    m_ekf_path.poses.push_back(ekf_pose);    
    
    ekf_path_pub.publish(m_ekf_path);

    transform_ekf.setOrigin(tf::Vector3(p_e.x, p_e.y, 0.0));
    q_e.setRPY(0, 0, m_x_post(2));
    transform_ekf.setRotation(q_e);
    tfcaster_ekf.sendTransform(tf::StampedTransform(transform_ekf, ros::Time::now(), "world", "EKF_frame"));
    
    m_ekf_odom.pose.pose.position = p_e;
    m_ekf_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_x_post(2));
    
    ekf_pose_pub.publish(m_ekf_odom);


};


void EKF::IMUPathVisualize(){
     // Path Line visualiazation
    geometry_msgs::PoseStamped imu_pose;
    
    m_imu_odom.header.frame_id = "world"; // Set the frame id
    m_imu_odom.header.stamp = ros::Time::now();

    m_imu_path.header.frame_id = "world"; // Set the frame id
    m_imu_path.header.stamp = ros::Time::now();

    p_i.x = m_imu_dr(0);
    p_i.y = m_imu_dr(1);
    p_i.z = 0;
    
    imu_pose.pose.position = p_i;
    imu_pose.pose.orientation.w = 1;

    m_imu_path.poses.push_back(imu_pose);    
    
    imu_path_pub.publish(m_imu_path);

    transform_imu.setOrigin(tf::Vector3(p_i.x, p_i.y, 0.0));
    q_e.setRPY(0, 0, m_imu_dr(2));
    transform_imu.setRotation(q_e);
    tfcaster_imu.sendTransform(tf::StampedTransform(transform_imu, ros::Time::now(), "world", "IMU_frame"));
    
    m_imu_odom.pose.pose.position = p_i;
    m_imu_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_imu_dr(2));
    
    imu_pose_pub.publish(m_imu_odom);


}; 


void EKF::UTMPathVisualize(){
      // Path Line visualiazation
    geometry_msgs::PoseStamped utm_pose;
    
    m_utm_odom.header.frame_id = "world"; // Set the frame id
    m_utm_odom.header.stamp = ros::Time::now();

    m_utm_path.header.frame_id = "world"; // Set the frame id
    m_utm_path.header.stamp = ros::Time::now();

    p_u.x = m_z_measured(0);
    p_u.y = m_z_measured(1);
    p_u.z = 0;
    
    utm_pose.pose.position = p_u;
    utm_pose.pose.orientation.w = 1;

    m_utm_path.poses.push_back(utm_pose);    
    
    utm_path_pub.publish(m_utm_path);

    transform_utm.setOrigin(tf::Vector3(p_u.x, p_u.y, 0.0));
    q_e.setRPY(0, 0, m_utm.yaw);
    transform_utm.setRotation(q_e);
    tfcaster_utm.sendTransform(tf::StampedTransform(transform_utm, ros::Time::now(), "world", "GPS_frame"));
    
    m_utm_odom.pose.pose.position = p_u;
    m_utm_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_utm.yaw);
    
    utm_pose_pub.publish(m_utm_odom);



};


int main(int argc, char **argv){

    ros::init(argc, argv, "EKF_node");
    EKF ekf;
    ros::Rate loop_rate(1 / ekf.m_dt);
    

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
