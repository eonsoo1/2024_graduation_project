#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"
#include <autonomous_msgs/PoseMsg.h>

#include "tf/tf.h"
#include "cmath"
#include <iostream>


#define Ld   1.0
#define PI   3.141592

using namespace std;

class Control{

private:

    ros::NodeHandle n;
    ros::Subscriber speed_sub;
    ros::Subscriber odometry_sub;
    ros::Publisher controller_pub;
    ros::Publisher vehicle_position_pub;
    ros::Publisher target_point_pub;
    ros::Publisher path_pub;

    string m_file_path;
    string m_path_folder_name;
    nav_msgs::Path m_global_path;
    std_msgs::Int32 m_steering;
    
    double m_velocity_ms;
    double m_vehicle_x;
    double m_vehicle_y;
    double m_vehicle_heading;

    double m_current_target_x;
    double m_current_target_y;

    double m_look_ahead_distance;
    double m_alpha;
    double m_wheel_base = 0.47;
    int index = 0;

    visualization_msgs::Marker line_strip;

public:
    Control();
    ~Control(){};

    void SpeedCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg);
    void EgoVehicleCallback(const autonomous_msgs::PoseMsg::ConstPtr &msg);

    void targetPoint();
    void VehicleCurrentPoint();
    void AlphaCalculator();
    double ControlCalculator();
    double ControlPid();
    // void PublishCommend(carla_msgs::CarlaEgoVehicleControl& msg);
    void Do();
    void NextWaypoint();
    void readCsv(const string &file_name);
};

/******************생성자*******************/

Control::Control(){
  m_file_path = ros::package::getPath("planning");
  path_pub = n.advertise<nav_msgs::Path>("/global_path", 1);
  speed_sub = n.subscribe("/ublox_gps/fix_velocity", 100, &Control::SpeedCallback, this);
  odometry_sub = n.subscribe("m_vehicle_pose", 100, &Control::EgoVehicleCallback, this);

  controller_pub = n.advertise<std_msgs::Int32>("/ctrl_cmd", 100);
  target_point_pub = n.advertise<visualization_msgs::Marker>( "waypoint", 1000 );
  vehicle_position_pub = n.advertise<visualization_msgs::Marker>( "vehicle_waypoint", 1);
    
}

/**********Speed 값 콜백 하는 함수***********/

void Control::SpeedCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& gps_velocity_msg){

  m_velocity_ms = sqrt(pow(gps_velocity_msg->twist.twist.linear.x, 2) + pow(gps_velocity_msg->twist.twist.linear.y, 2)); // (m/s 단위)
  
}

/*************차량의 현재 값 받는 함수**************/
void Control::EgoVehicleCallback(const autonomous_msgs::PoseMsg::ConstPtr &msg){
    

    m_vehicle_x = msg->x;
    m_vehicle_y = msg->y;
    m_vehicle_heading = msg->heading * M_PI / 180;

}

void Control::readCsv(const string &file_name){
    m_path_folder_name = "path";
    string full_file_name = m_file_path + "/" + m_path_folder_name + "/" + file_name;
    ifstream openFile(full_file_name);

    m_global_path.header.frame_id = "/map";
    int count =0;

    if(openFile.is_open()){

        string line;

        while(getline(openFile, line)){
            // cout<<count<<endl;
            vector<string> row;
            stringstream ss(line);
            string cell;

            while(getline(ss, cell, ',')){
                row.push_back(cell);
            }

            geometry_msgs::PoseStamped read_pose;
            read_pose.pose.position.x = stof(row[0]);
            read_pose.pose.position.y = stof(row[1]);
            read_pose.pose.position.z = 0;
            read_pose.pose.orientation.x = 0;
            read_pose.pose.orientation.y = 0;
            read_pose.pose.orientation.z = 0;
            read_pose.pose.orientation.w = 0;
            m_global_path.poses.push_back(read_pose);
            // cout<<read_pose<<endl;           
            
        }

        openFile.close();
        //publish global path 또는 global path에 저장
    }

    else{
        ROS_ERROR("Unable to open file");
    }
    path_pub.publish(m_global_path);

}

/*************현재 바라보고 있는 target point visualiazation**************/
void Control::targetPoint(){

  visualization_msgs::Marker marker;

	marker.header.frame_id = "world";

	marker.header.stamp = ros::Time();

  /**********************마커의 네임스페이스************************/
	marker.ns = "my_namespace";

  /********************* 마커에 할당된 고유 ID**********************/
	marker.id = 0;

  /*********The available types are specified in the message definition.********/
	marker.type = visualization_msgs::Marker::CUBE;

  /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
	marker.action = visualization_msgs::Marker::ADD;

  /****************Pose marker, specified as x/y/z position ***************/
	marker.pose.position.x = m_current_target_x;
	marker.pose.position.y = m_current_target_y;


  /******************* x/y/z/w quaternion orientation.*********************/
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0; //쿼터니언 값(?)
	
  /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
    marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;

  /***************************객체의 색상*****************************/
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	//only if using a MESH_RESOURCE marker type:
	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  
	target_point_pub.publish( marker );

}

/*************현재 내 차량의 위치 visualiazation**************/

void Control::VehicleCurrentPoint(){

	line_strip.header.frame_id = "map";

	line_strip.header.stamp = ros::Time::now();

  /**********************마커의 네임스페이스************************/
	line_strip.ns = "line_strip";

  /********************* 마커에 할당된 고유 ID**********************/
	line_strip.id = 0;

  /*********The available types are specified in the message definition.********/
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
	line_strip.action = visualization_msgs::Marker::ADD;

  /******************* x/y/z/w quaternion orientation.*********************/
	line_strip.pose.orientation.x = 0.0;
	line_strip.pose.orientation.y = 0.0;
	line_strip.pose.orientation.z = 0.0;
	line_strip.pose.orientation.w = 1.0; //쿼터니언 값(?)
	
  /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
  line_strip.scale.x = 0.3;
	

  /***************************객체의 색상*****************************/
	line_strip.color.a = 1.0; // Don't forget to set the alpha!
	line_strip.color.r = 1.0;
	line_strip.color.g = 1.0;
	line_strip.color.b = 0.0;

	//only if using a MESH_RESOURCE marker type:
	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";


 /****************Pose marker, specified as x/y/z position ***************/

  geometry_msgs::Point p1;
  
  p1.x = m_vehicle_x;
  p1.y = m_vehicle_y;
  
  line_strip.points.push_back(p1);

	vehicle_position_pub.publish(line_strip);

}

/***********다음 target point 찾는 함수************/

void Control::NextWaypoint(){

  double ld_next = sqrt(pow((m_global_path.poses[index].pose.position.x-m_vehicle_x), 2) + pow((m_global_path.poses[index].pose.position.y-m_vehicle_y), 2));

  if(ld_next >= Ld){
    m_current_target_x = m_global_path.poses[index].pose.position.x;
    m_current_target_y = m_global_path.poses[index].pose.position.y;

    // std::cout << "-----next_ld = "<< ld_next <<" 다음 point를 찾았습니다.-----" << std::endl;
    // std::cout << " Next_Target_x = " << m_current_target_x << std::endl;
    // std::cout << " Next_Target_y = " << m_current_target_y << std::endl; 
  }
  while(ld_next < Ld){

    index++;

    m_current_target_x = m_global_path.poses[index].pose.position.x;
    m_current_target_y = m_global_path.poses[index].pose.position.y;

    ld_next = sqrt(pow((m_current_target_x-m_vehicle_x), 2) + pow((m_current_target_y-m_vehicle_y), 2)); 

    // std::cout << "-----next_ld = "<< ld_next <<" 다음 point를 찾는 중입니다.-----" << std::endl;
    // std::cout << " Next_Target_x = " << m_current_target_x << std::endl;
    // std::cout << " Next_Target_y = " << m_current_target_y << std::endl;
  }
  
}

/****************차와 target point 사이의 각도 구하는 함수*****************/

void Control::AlphaCalculator(){
  
  m_alpha = atan2((m_current_target_y - m_vehicle_y), (m_current_target_x - m_vehicle_x)) ;
  m_look_ahead_distance = sqrt(pow((m_current_target_x - m_vehicle_x), 2) + pow((m_current_target_y - m_vehicle_y), 2));

}

int mapValue(double value, double minFrom, double maxFrom, int minTo, int maxTo) {
            // 원래 범위에서의 비율 계산
            double ratio = (value - minFrom) / (maxFrom - minFrom);
            
            // 새로운 범위에서의 값 계산
            int result = static_cast<int>(minTo + ratio * (maxTo - minTo) + 0.5); // 반올림
            
            // 결과 값이 새로운 범위를 넘어가지 않도록 보정
            return std::min(maxTo, std::max(minTo, result));
        }

/**************Control Pursuit 알고리즘으로 steer 구하는 함수*****************/

double Control::ControlCalculator(){
  
    AlphaCalculator();
    double steering_angle = -atan2(2.0 * m_wheel_base * sin(m_alpha - m_vehicle_heading), m_look_ahead_distance);

    m_steering.data = mapValue(steering_angle, -1.0, 1.0, 128, 255);
    cout << " Alpha : " << m_alpha << endl;
    cout << " Steering : " << steering_angle << endl;
    cout << " Mapped Steering : " << m_steering.data << endl;

    controller_pub.publish(m_steering);

    if(m_look_ahead_distance > Ld){// 멀리 있으면 가까이 올 때까지 대기
        // std::cout << "-------------해당 waypoint를 따라갑니다.-------------" << std :: endl;
        // std::cout << " Index : " << index << std :: endl;
        // std::cout << "" << std::endl;  
        // std::cout << "" << std::endl;  
        // std::cout << "" << std::endl;  
    }
    // std::cout << "-------------다음 waypoint를 찾습니다.--------------" << std :: endl;
    NextWaypoint();
    // std::cout << " Index : " << index << std :: endl;


    return steering_angle;

}


/*************PID 제어**************/

double Control::ControlPid(){

  double setspeed = 35;
  
  
  double error = setspeed - m_velocity_ms;
  double previous_error = 0;
  std::cout<< " m_velocity_ms(km/h) = " << m_velocity_ms << std::endl;
  double kp = 0.1;
  double ki = 0.2;
  double kd = 0.05;

  double dt = 0.1;
  double intergral; 
  double differential;

  intergral = intergral + error * dt;
  differential = error - previous_error / dt;
  
  m_velocity_ms = kp * error + ki * intergral - kd * differential;
  if (m_velocity_ms > 1){
      m_velocity_ms = 1 ;
  } 
  previous_error = error;
  

  return m_velocity_ms;

}

void Control::Do(){

    targetPoint();
    ControlCalculator();
    std::cout << " Vehicle_yaw = " << m_vehicle_heading * 180 / M_PI << std::endl;
    std::cout << " Current_Target_x = " << m_current_target_x << std::endl;
    std::cout << " Current_Target_y = " << m_current_target_y << std::endl;
    std::cout << " Current_Vehicle_x = " << m_vehicle_x << std::endl;
    std::cout << " Current_Vehicle_y = " << m_vehicle_y << std::endl; 

    std::cout<< " Look_Ahead_Distance = " << m_look_ahead_distance << std::endl;
    std::cout << "-------------------------" << std::endl;  
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "pure_pursuit");
    Control ctrl;
    ros::Rate loop_rate(80);
    
    ctrl.readCsv("test.csv");

    while(ros::ok()){
        
        ctrl.Do();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}