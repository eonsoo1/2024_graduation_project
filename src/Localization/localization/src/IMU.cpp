#include "localization/IMU.hpp"


Deadreckoning::Deadreckoning(){
    
    imu_sub = nh.subscribe("/imu/data", 1000, &Deadreckoning::ImuCallback, this);
    utm_coord_sub = nh.subscribe("/utm_coord", 1000, &Deadreckoning::UTMCallback, this);
    gps_velocity_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &Deadreckoning::GPSVelocityCallback, this);
    z_calibration_velocity_pub  = nh.advertise<std_msgs::Float32>("z_calib_velocity", 1000);
    imu_pose_pub = nh.advertise<localization::PoseMsg>("/imu_pose", 1000);
    start_time = std::chrono::steady_clock::now();
    // m_collet_time = true;
    m_utm_bool = false;
    m_gps_vel_bool = false;
    m_gps_yaw_trigger = false;
    m_prev_velocity = 0;
    nh.param("init_yaw", m_init_yaw, INITIAL_YAW);
};

void Deadreckoning::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_coord_msg){

    if(!m_utm_bool){
        m_utm_x = utm_coord_msg->x;
        m_utm_y = utm_coord_msg->y;
        m_origin_x = m_utm_x;
        m_origin_y = m_utm_y;
       
        marker_pub = nh.advertise<visualization_msgs::Marker>("/imu_dr_path", 1000);
        imu_path.header.frame_id = "world"; // Set the frame id
        imu_path.header.stamp = ros::Time::now();
        imu_path.ns = "line_strip";
        imu_path.action = visualization_msgs::Marker::ADD;
        imu_path.type = visualization_msgs::Marker::LINE_STRIP;
        imu_path.pose.orientation.x = 0;
        imu_path.pose.orientation.y = 0;
        imu_path.pose.orientation.z = 0;
        imu_path.pose.orientation.w = 1.0;
        imu_path.scale.x = 0.3; // Line width

        // Set the marker color (RGBA)
        imu_path.color.r = 0.3;
        imu_path.color.g = 0.5;
        imu_path.color.b = 0.0;
        imu_path.color.a = 1.0;
        p.x = m_origin_x;
        p.y = m_origin_y;
        p.z = 0;

       

        transform.setOrigin(tf::Vector3(p.x, p.y, 0.0));
        q1.setRPY(0, 0, 0);
        transform.setRotation(q1);
        tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu_frame"));

        imu_path.points.push_back(p);

        m_utm_bool =true;
    }
    else{
        m_utm_x = utm_coord_msg->x;
        m_utm_y = utm_coord_msg->y;
        double distance;
        distance = sqrt(pow((m_utm_x - m_origin_x), 2) + pow((m_utm_y - m_origin_y), 2));
        if(distance > 0.1 && !m_gps_yaw_trigger){
            m_gps_yaw = atan2((m_utm_y - m_origin_y), (m_utm_x - m_origin_x));
            m_gps_yaw_trigger = true;
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
    if(m_utm_bool && m_gps_vel_bool){
        m_delta_time = 0.01;
        //초기값 설정 단계
        if (initial_time) {
            if(m_dVehicleVel_ms < 0.1){
                    auto current_time = chrono::steady_clock::now();
                    auto elapsed_time = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
                    cout << "--------------------" << endl;
                    cout << "데이터 수집 중 : " << elapsed_time << "초"<< endl;
                    CollectCalibrationData(calibration_velocity_data, imu_data_msg->angular_velocity);
            }
            else {
                m_delta_time = 0; // 시간 간격을 0으로 설정합니다.
                CalibrateSensor(calibration_velocity_data, calibration_velocity_offsets);
                // CalcOrientation(imu_data_msg->orientation); // 초기값 정해주는 문장 이후 다시 설정 예정
                m_prev_velocity = 0;
                m_imu_yaw = m_init_yaw * M_PI / 180; //convert degree to radian
                initial_time = false; // 첫 번째 시간 플래그를 해제합니다.
                cout << "--------------------" << endl;
                cout<< "Initial yaw(degree) : " << m_imu_yaw << endl;
            } 
        }
        //초기값 설정 이후 
        else if(m_gps_yaw_trigger){
            IMUDeadReckoning(imu_data_msg->angular_velocity,imu_data_msg->linear_acceleration);
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
    calibration_offsets.x = -average_x;
    calibration_offsets.y = -average_y;
    calibration_offsets.z = -average_z; // 9.8은 중력의 기대값

    std::cout << "보정 오프셋: [" << calibration_offsets.x << ", " << calibration_offsets.y << ", " << calibration_offsets.z << "]\n";
}


void Deadreckoning::Pub(){
   
    localization::PoseMsg imu_pose;

    imu_pose.pose_x = m_imu_x;
    imu_pose.pose_y = m_imu_y;
    imu_pose.pose_yaw = m_imu_yaw;
    imu_pose.yaw_rate = m_yaw_rate;

    imu_pose_pub.publish(imu_pose);
    marker_pub.publish(imu_path);
    z_calibration_velocity_pub.publish(calib_velocity_z);
};

void Deadreckoning::CalcOrientation(const geometry_msgs::Quaternion &msg){
    orientation = msg;
    double roll, pitch, yaw;
    tf::Quaternion odom_quat(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ); 
    tf::Matrix3x3 m(odom_quat);
    m.getRPY(roll, pitch, yaw);
   
    m_imu_yaw = yaw * 180 / M_PI;

}

void Deadreckoning::IMUDeadReckoning(const geometry_msgs::Vector3 &velocity_msg, 
                                            const geometry_msgs::Vector3 &accel_msg) {                                     
    m_yaw_rate = (velocity_msg.z) ; // 각속도 보정
    calib_velocity_z.data = m_yaw_rate;
    
    double dx = 0.0;
    double dy = 0.0;
    double dyaw = m_yaw_rate * m_delta_time; // yaw의 각속도는 z축을 기준으로하는 회전 속도를 말함
    
    // m_dVehicleVel_ms = m_prev_velocity + sqrt(pow((accel_msg.x * m_delta_time), 2) + pow((accel_msg.y * m_delta_time), 2));
    // m_dVehicleVel_ms = (m_prev_velocity) + sqrt(pow((accel_msg.x), 2) + pow((accel_msg.y), 2)) * m_delta_time;
    
    m_prev_velocity = m_dVehicleVel_ms;

    dx = m_dVehicleVel_ms * m_delta_time * cos(m_imu_yaw);
    dy = m_dVehicleVel_ms * m_delta_time * sin(m_imu_yaw);

    m_imu_x += dx;
    m_imu_y += dy;
    m_imu_yaw += dyaw;

    //launch 실행 시 발산 하는 x, y로 인한 line strip 오류 발생을 막기 위한 조건문
    if(m_imu_x > 9999. || m_imu_x < -9999.){
        cout << "IMU X : " << m_imu_x << endl;
        m_imu_x = 0;
    }
    if(m_imu_y > 9999. || m_imu_y < -9999.){
        cout << "IMU Y : " << m_imu_y << endl;
        m_imu_y = 0;
    }
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


    imu_path.points.push_back(p);

}


int main (int argc, char** argv){

	ros::init(argc, argv, "IMU_node");
	Deadreckoning imu;
    ros::Rate loop_rate(100);
    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();

    }


}