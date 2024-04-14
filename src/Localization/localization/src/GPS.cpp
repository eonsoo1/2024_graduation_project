#include "localization/GPS.hpp"

GPS::GPS(){
    utm_pub = nh.advertise<geometry_msgs::Point>("/utm_coord", 1000);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/gps_path", 1000);
    gps_sub = nh.subscribe("/ublox_gps/fix", 1000, &GPS::GPSCallback, this);
    gps_bool = false;
    // m_origin = lanelet::Origin({ORIGIN_LAT, ORIGIN_LON}); // 삼각지 기준 rviz mapping
 }

void GPS::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg){

    m_lat = gps_data_msg->latitude;
    m_lon = gps_data_msg->longitude;
    m_alt = gps_data_msg->altitude;  

    // 시작점 기준 rviz mapping
    if(!gps_bool){
        m_origin = lanelet::Origin({m_lat, m_lon});
        gps_bool = true;
    }
    
    m_gps_point.lat = m_lat;
    m_gps_point.lon = m_lon;
    lanelet::projection::UtmProjector projection(m_origin);
    
    m_utm_point.x() = projection.forward(m_gps_point).x();
    m_utm_point.y() = projection.forward(m_gps_point).y();
    

    gps_path.header.frame_id = "world"; // marker의 좌표 프레임 설정
    gps_path.header.stamp = ros::Time::now();
    gps_path.ns = "gps_marker";
    gps_path.id = 0;
    gps_path.type = visualization_msgs::Marker::LINE_STRIP; // 선 모양의 marker
    gps_path.action = visualization_msgs::Marker::ADD;
    gps_path.pose.orientation.x = 0;
    gps_path.pose.orientation.y = 0;
    gps_path.pose.orientation.z = 0;
    gps_path.pose.orientation.w = 1.0;
    gps_path.scale.x = 0.3;
    gps_path.scale.y = 0.3; 
    gps_path.scale.z = 0.3;  // marker 크기 설정
    gps_path.color.a = 1.0; // 투명도
    gps_path.color.r = 1.0; // 빨간색
    gps_path.color.g = 0.0;
    gps_path.color.b = 0.0;

    // marker의 점들을 추가합니다.

    m_utm_coord.x = m_utm_point.x();
    m_utm_coord.y = m_utm_point.y();
    m_utm_coord.z = 0;


    tf::Transform transform;
    tf::Quaternion q1;

    transform.setOrigin(tf::Vector3(m_utm_coord.x, m_utm_coord.y, 0.0));
    q1.setRPY(0, 0, 0);
    transform.setRotation(q1);
    tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gps_frame"));


    gps_path.points.push_back(m_utm_coord); // 점 추가

    // 발행된 marker를 rviz에 표시합니다.
    marker_pub.publish(gps_path);
    utm_pub.publish(m_utm_coord);
}


void GPS::Print(){
    cout << "-----------" << endl;
    ROS_INFO("latitude : %f", m_lat);
    ROS_INFO("longitude : %f", m_lon);
    ROS_INFO("altitude : %f", m_alt);
    ROS_INFO("UTM POINT_X : %f", m_utm_point.x());
    ROS_INFO("UTM POINT_Y : %f", m_utm_point.y());

}


int main(int argc, char **argv){

    ros::init(argc, argv, "GPS_node");
    
    GPS gps;
 
    ros::Rate loop_rate(8);
    while(ros::ok()){

        gps.Print();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}
