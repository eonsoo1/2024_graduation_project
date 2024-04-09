#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

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

using namespace std;

class EKF{
    private:
        ros::NodeHandle nh;
        ros::Subscriber utm_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;

        double m_x_covariance;
        double m_y_covariance;
        double m_utm_x;
        double m_utm_y;
        double m_imu_x;
        double m_imu_y;
        double m_imu_yaw;


    public:
        EKF();
        ~EKF(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg);
        void IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg);
};

EKF::EKF(){

    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &EKF::GPSCallback, this);
    utm_sub = nh.subscribe("/utm_coord", 1000, &EKF::UTMCallback, this);
    imu_sub = nh.subscribe("/imu_pose", 1000, &EKF::IMUCallback, this);

};

void EKF::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg){
  
    m_x_covariance = gps_data_msg->position_covariance[0]; //1행 1열 x covariance
    m_y_covariance = gps_data_msg->position_covariance[4]; //2행 2열 y covariance

};

void EKF::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_data_msg){
    
    m_utm_x = utm_data_msg->x;
    m_utm_y = utm_data_msg->y;
    
};

void EKF::IMUCallback(const localization::PoseMsg::ConstPtr& imu_data_msg){
    
    m_imu_x = imu_data_msg->pose_x;
    m_imu_y = imu_data_msg->pose_y;
    m_imu_yaw = imu_data_msg->pose_yaw;

};


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
