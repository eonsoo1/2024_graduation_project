#include "localization/IMU.hpp"


Deadreckoning::Deadreckoning(){
    
    utm_coord_sub = nh.subscribe("/utm_coord", 1000, &Deadreckoning::UTMCallback, this);
    gps_velocity_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &Deadreckoning::GPSVelocityCallback, this);
    imu_sub = nh.subscribe("/imu/data", 1000, &Deadreckoning::ImuCallback, this);
    imu_pose_pub = nh.advertise<localization::PoseMsg>("/imu_pose", 1000);
    imu_path_pub = nh.advertise<nav_msgs::Path>("/imu_dr_path", 1000);
    z_calibration_velocity_pub  = nh.advertise<std_msgs::Float32>("z_calib_velocity", 1000);
    // m_collet_time = true;
    m_utm_bool = false;
    m_gps_vel_bool = false;
    m_gps_yaw_trigger = false;
    initial_time = true;  
    m_prev_velocity = 0;
    // nh.param("init_yaw", m_init_yaw, INITIAL_YAW);
};

void Deadreckoning::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_coord_msg){
    if(!m_gps_yaw_trigger){
        if(!m_utm_bool){
            m_utm_x = utm_coord_msg->x;
            m_utm_y = utm_coord_msg->y;
            m_origin_x = m_utm_x;
            m_origin_y = m_utm_y;
          

            geometry_msgs::PoseStamped imu_pose_stamp;
            
            m_imu_path.header.frame_id = "world"; // Set the frame id
            m_imu_path.header.stamp = ros::Time::now();
        
            p.x = m_origin_x;
            p.y = m_origin_y;
            p.z = 0;

            imu_pose_stamp.pose.position = p;
            imu_pose_stamp.pose.orientation.w = 1;    

            m_imu_path.poses.push_back(imu_pose_stamp);
            imu_path_pub.publish(m_imu_path);    

            transform.setOrigin(tf::Vector3(p.x, p.y, 0.0));
            q1.setRPY(0, 0, 0);
            transform.setRotation(q1);
            tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu_frame"));

            m_utm_bool =true;
        }
        else{
            m_utm_x = utm_coord_msg->x;
            m_utm_y = utm_coord_msg->y;
           
            double distance;
            distance = sqrt(pow((m_utm_x - m_origin_x), 2) + pow((m_utm_y - m_origin_y), 2));
            if(m_dVehicleVel_ms > 0.1 && distance > 3.0 && !m_gps_yaw_trigger){//0.1
                m_imu_x = m_utm_x;
                m_imu_y = m_utm_y;
                m_gps_yaw = atan2((m_utm_y - m_origin_y), (m_utm_x - m_origin_x));
                m_gps_yaw_trigger = true;
            }
        }
    }

}

void Deadreckoning::GPSVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
   
    m_velocity_x = gps_velocity_msg->twist.twist.linear.x;
    m_velocity_y = gps_velocity_msg->twist.twist.linear.y;
    m_velocity_z = gps_velocity_msg->twist.twist.linear.z;
    m_dVehicleVel_ms = sqrt(pow(m_velocity_x, 2) + pow(m_velocity_y, 2));  
    m_gps_vel_bool = true;
}

void Deadreckoning::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg){
    if(m_utm_bool && m_gps_yaw_trigger){ //gps_vel_bool -> gps_yaw_trigger
        
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
            IMUDeadReckoning(imu_data_msg->angular_velocity);
            Pub();
        }
    }

    if(initial_time) {
            if(m_dVehicleVel_ms < 0.01){
                    cout << "--------------------" << endl;
                    cout << "데이터 수집 중" << endl;
                    CollectCalibrationData(calibration_velocity_data, imu_data_msg->angular_velocity);
            }
        }
};

void Deadreckoning::CollectCalibrationData(std::vector<geometry_msgs::Vector3>& calibration_data, const geometry_msgs::Vector3& msg) {

    calibration_data.push_back(msg);

};


void Deadreckoning::CalibrateSensor(const std::vector<geometry_msgs::Vector3>& calibration_data,
                            geometry_msgs::Vector3& calibration_offsets) {
        // 데이터의 평균값 계산
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto& data : calibration_data) {
        sum_x += data.x;
        sum_y += data.y;
        sum_z += data.z;
        
    }
    double average_x = sum_x / calibration_data.size();
    double average_y = sum_y / calibration_data.size();
    double average_z = sum_z / calibration_data.size();

    // 보정 오프셋 설정하여 편향 보정
    calibration_offsets.x = average_x;
    calibration_offsets.y = average_y;
    calibration_offsets.z = average_z; 
    
    std::cout << "보정 오프셋: [" << calibration_offsets.x << ", " << calibration_offsets.y << ", " << calibration_offsets.z << "]\n";
}


void Deadreckoning::Pub(){
   
    localization::PoseMsg imu_pose;

    imu_pose.pose_x = m_imu_x;
    imu_pose.pose_y = m_imu_y;
    imu_pose.pose_yaw = m_imu_yaw;
    imu_pose.yaw_rate = m_yaw_rate;

    imu_pose_pub.publish(imu_pose);
    imu_path_pub.publish(m_imu_path);
    z_calibration_velocity_pub.publish(calib_velocity_z);
};


void Deadreckoning::IMUDeadReckoning(const geometry_msgs::Vector3 &velocity_msg) {                                     
    m_yaw_rate = (velocity_msg.z - calibration_velocity_offsets.z) ; // 각속도 보정
    calib_velocity_z.data = m_yaw_rate;

    double dx = 0.0;
    double dy = 0.0;
    double dyaw = m_yaw_rate * m_delta_time; // yaw의 각속도는 z축을 기준으로하는 회전 속도를 말함
    
    m_prev_velocity = m_dVehicleVel_ms;

    dx = m_dVehicleVel_ms * m_delta_time * cos(m_imu_yaw);
    dy = m_dVehicleVel_ms * m_delta_time * sin(m_imu_yaw);

    m_imu_x += dx;
    m_imu_y += dy;
    m_imu_yaw += dyaw;

    // //launch 실행 시 발산 하는 x, y로 인한 line strip 오류 발생을 막기 위한 조건문
    // if(m_imu_x > 9999. || m_imu_x < -9999.){
    //     cout << "IMU X : " << m_imu_x << endl;
    //     m_imu_x = 0;
    // }
    // if(m_imu_y > 9999. || m_imu_y < -9999.){
    //     cout << "IMU Y : " << m_imu_y << endl;
    //     m_imu_y = 0;
    // }

    if(m_imu_yaw > 2 * M_PI){
        m_imu_yaw = 0;
    }
    if(m_imu_yaw < - 2 * M_PI){
        m_imu_yaw = 0;
    }
    
    geometry_msgs::PoseStamped imu_pose_stamp;
    Eigen::VectorXd p_before(2), p_after(2);
    Eigen::MatrixXd R(2,2);
    
    p_before << m_imu_x, 
                m_imu_y;
    
    R << cos(m_gps_yaw), -sin(m_gps_yaw),
        sin(m_gps_yaw), cos(m_gps_yaw);
    
    p_after = R * p_before;

    p.x = p_after(0);
    p.y = p_after(1);
    p.z = 0;

    imu_pose_stamp.pose.position = p;
    imu_pose_stamp.pose.orientation.w = 1;

    m_imu_path.poses.push_back(imu_pose_stamp);    
    // p.x = m_imu_x;
    // p.y = m_imu_y;
    // p.z = 0;

    cout << "-------------" << endl;
    cout << "IMU X : " <<  p_after(0) << endl;
    cout << "IMU Y : " << p_after(1) << endl;
    cout << "IMU Yaw : " << m_imu_yaw * 180 / M_PI << endl;
    cout << "Velocity : " << m_dVehicleVel_ms << endl;

   
    transform.setOrigin(tf::Vector3(p.x, p.y, 0.0));
    q1.setRPY(0, 0, 0);
    transform.setRotation(q1);
    tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu_frame"));
    

}


int main (int argc, char** argv){

	ros::init(argc, argv, "IMU_node");
	Deadreckoning imu;
    ros::Rate loop_rate(1 / imu.m_delta_time);
    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();

    }


}