#include "localization/IMU.hpp"


Deadreckoning::Deadreckoning(){
    

    gps_velocity_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &Deadreckoning::GPSVelocityCallback, this);
    imu_sub = nh.subscribe("/imu/data", 1000, &Deadreckoning::ImuCallback, this);
    imu_pose_pub = nh.advertise<localization::PoseMsg>("/imu_pose", 1000);
  
    // z_calibration_velocity_pub  = nh.advertise<std_msgs::Float32>("z_calib_velocity", 1000);
    // m_collet_time = true;
    m_utm_bool = false;
    m_gps_vel_bool = false;
    m_gps_yaw_trigger = false;
    initial_time = true;  
    data_collect = false;
    m_prev_velocity = 0;
    m_dVehicleVel_ms = 0;
    // nh.param("init_yaw", m_init_yaw, INITIAL_YAW);
};


void Deadreckoning::GPSVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
   
    m_velocity_x = gps_velocity_msg->twist.twist.linear.x;
    m_velocity_y = gps_velocity_msg->twist.twist.linear.y;
    m_velocity_z = gps_velocity_msg->twist.twist.linear.z;
    m_dVehicleVel_ms = sqrt(pow(m_velocity_x, 2) + pow(m_velocity_y, 2));  

}

void Deadreckoning::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg){

if(!data_collect) {
    if(m_dVehicleVel_ms < 0.1){
            cout << "--------------------" << endl;
            cout << "데이터 수집 중" << endl;
            cout << "Velocity : " << m_dVehicleVel_ms << endl;
            CollectCalibrationData(calibration_velocity_data, imu_data_msg->angular_velocity);
    }
    else{
        data_collect = true;
    }
}
else{
        //초기값 설정 단계
    if(initial_time) {
            CalibrateSensor(calibration_velocity_data, calibration_velocity_offsets);
            m_prev_velocity = 0;
            m_imu_yaw = m_init_yaw * M_PI / 180; //convert degree to radian
            cout << "--------------------" << endl;
            cout<< "Initial yaw(degree) : " << m_imu_yaw << endl;
            initial_time = false; // 첫 번째 시간 플래그를 해제합니다.
        
    }
    //초기값 설정 이후 
    else{
        // m_yaw_rate = imu_data_msg->angular_velocity.z - calibration_velocity_offsets.z; // 각속도 보정 o
        m_yaw_rate = imu_data_msg->angular_velocity.z; // 각속도 보정 x
        Pub();
    }
}
};

void Deadreckoning::CollectCalibrationData(std::vector<geometry_msgs::Vector3>& calibration_data, const geometry_msgs::Vector3& msg) {

    calibration_data.push_back(msg);

};


void Deadreckoning::CalibrateSensor(const std::vector<geometry_msgs::Vector3>& calibration_data,
                            geometry_msgs::Vector3& calibration_offsets) {
        // 데이터의 평균값 계산
    double  sum_z = 0.0;
    for (const auto& data : calibration_data) {

        sum_z += data.z;
    }

    double average_z = sum_z / calibration_data.size();

    calibration_offsets.z = average_z; 
    
    std::cout << "보정 오프셋: ["  << calibration_offsets.z << "]\n";
}


void Deadreckoning::Pub(){
   
    localization::PoseMsg imu_pose;
    
    imu_pose.yaw_rate = m_yaw_rate;
    cout << "--------------------" << endl;
    cout << "Yaw Rate : " << m_yaw_rate << endl;
    cout << "Velocity : " << m_dVehicleVel_ms << endl;
    imu_pose_pub.publish(imu_pose);
    // imu_path_pub.publish(m_imu_path);
    // z_calibration_velocity_pub.publish(calib_velocity_z);
};


int main (int argc, char** argv){

	ros::init(argc, argv, "IMU_node");
	Deadreckoning imu;
    ros::Rate loop_rate(1 / imu.m_delta_time);
    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();

    }


}