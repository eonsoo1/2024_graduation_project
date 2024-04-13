#include "localization/imu_deadreckoning.hpp"


Deadreckoning::Deadreckoning(){
    
    imu_sub = nh.subscribe("/imu/data", 1000, &Deadreckoning::ImuCallback, this);
    utm_coord_sub = nh.subscribe("/utm_coord", 1000, &Deadreckoning::UTMCallback, this);
    gps_velocity_sub = nh.subscribe("/ublox_gps/fix_velocity", 1000, &Deadreckoning::GPSVelocityCallback, this);
    z_calibration_velocity_pub  = nh.advertise<std_msgs::Float32>("z_calib_velocity", 1000);
    imu_pose_pub = nh.advertise<localization::PoseMsg>("/imu_pose", 1000);
    start_time = std::chrono::steady_clock::now();
    m_collet_time = true;
    m_utm_msg = false;
    m_gps_yaw_trigger = false;
    m_prev_velocity = 0;
    nh.param("init_yaw", m_init_yaw, INITIAL_YAW);
};

void Deadreckoning::UTMCallback(const geometry_msgs::Point::ConstPtr& utm_coord_msg){

    if(!m_utm_msg){
        m_utm_x = utm_coord_msg->x;
        m_utm_y = utm_coord_msg->y;
        m_origin_x = m_utm_x;
        m_origin_y = m_utm_y;
        m_prev_utm_x = m_origin_x;
        m_prev_utm_y = m_origin_y;
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
        imu_path.points.push_back(p);

        m_utm_msg =true;
    }
    else{
        m_utm_x = utm_coord_msg->x;
        m_utm_y = utm_coord_msg->y;
        if(!m_gps_yaw_trigger){
            m_gps_yaw = atan2((m_utm_y - m_prev_utm_y), (m_utm_x - m_prev_utm_x));
            m_gps_yaw_trigger = true;
        }
    }


}

void Deadreckoning::GPSVelocityCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){
    m_velocity_x = gps_velocity_msg->twist.twist.linear.x;
    m_velocity_y = gps_velocity_msg->twist.twist.linear.y;
    m_velocity_z = gps_velocity_msg->twist.twist.linear.z;
}

void Deadreckoning::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_data_msg){
    if(m_utm_msg){

        
        // if(m_collet_time){
        //     auto current_time = chrono::steady_clock::now();
        //     auto elapsed_time = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
        //     cout << "-------"<< endl;
        //     cout << "데이터 수집 중 : " <<elapsed_time << "초"<< endl;
        //     CollectCalibrationData(calibration_velocity_data, imu_data_msg->angular_velocity);
            
        //     if(elapsed_time >= 10){
        //         m_collet_time = false;
        //         initial_time = true;
        //     }
        //     this_thread::sleep_for(chrono::milliseconds(100));
        // }
        
        // else 
        if (initial_time) {
            time = imu_data_msg->header.stamp; // 처음으로 메시지가 도착한 시간을 설정합니다.
            m_delta_time = 0; // 시간 간격을 0으로 설정합니다.
            CalibrateAccelerometer(calibration_velocity_data, calibration_velocity_offsets);
            CalcOrientation(imu_data_msg->orientation);
            m_prev_velocity = 0;
            m_imu_yaw = m_init_yaw;
            initial_time = false; // 첫 번째 시간 플래그를 해제합니다.
            // m_imu_yaw = m_imu_yaw * 180/M_PI;
            cout << "-------------" << endl;
            cout<< "Initial yaw(degree) : " << m_imu_yaw << endl;
        } 
        else {
            // m_delta_time = (imu_data_msg->header.stamp - time).toSec(); // 시간 간격을 설정합니다.
            m_delta_time = 0.01;
            time = imu_data_msg->header.stamp; // 현재 시간을 갱신합니다.
            IMUDeadReckoning(imu_data_msg->angular_velocity,imu_data_msg->linear_acceleration);
            Pub();

        }
    }
};

void Deadreckoning::CollectCalibrationData(std::vector<geometry_msgs::Vector3>& calibration_data, const geometry_msgs::Vector3& msg) {

    calibration_data.push_back(msg);

};


void Deadreckoning::CalibrateAccelerometer(const std::vector<geometry_msgs::Vector3>& calibration_data,
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
    m_dVehicleVel_ms = sqrt(pow(m_velocity_x, 2) + pow(m_velocity_y, 2));  
   
    m_prev_velocity = m_dVehicleVel_ms;

    dx = m_dVehicleVel_ms * m_delta_time * cos(m_imu_yaw);
    dy = m_dVehicleVel_ms * m_delta_time * sin(m_imu_yaw);

    m_imu_x += dx;
    m_imu_y += dy;
    m_imu_yaw += dyaw;

    if(m_imu_x > 9999. || m_imu_x < -9999.){
        cout << "IMU X : " << m_imu_x << endl;
        m_imu_x = 0;
    }
    
    p.x = m_origin_x - m_imu_x;
    p.y = m_origin_y - m_imu_y;
    p.z = 0;

    cout << "-------------" << endl;
    cout << "X : " << m_imu_x << endl;
    cout << "Y : " << m_imu_y << endl;
    cout << "Yaw : " << m_imu_yaw << endl;
    cout << "Velocity : " << m_dVehicleVel_ms << endl;

    // tf::Transform transform;

    // transform.setOrigin(tf::Vector3(m_utm_x, m_utm_y, 0.0));
    // tf::Quaternion q1;
    // q1.setRPY(0, 0, m_imu_yaw);
    // transform.setRotation(q1);

    // tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dr_frame"));
  
    imu_path.points.push_back(p);

}


int main (int argc, char** argv){

	ros::init(argc, argv, "imu_deadreckoning_node");
	Deadreckoning imu;
    ros::Rate loop_rate(100);
    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();

    }


}