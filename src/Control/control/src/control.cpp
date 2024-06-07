#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <string>
#include <thread>
#include <cmath>

#define Kp  0.
#define Ki  0.
#define Kd  0.

using namespace std;

class Control{
    private:
        ros::NodeHandle nh;
        ros::Publisher control_data_pub;
        ros::Subscriber speed_sub;

        int m_throttle;
        int m_steering;
        double m_vehicle_speed = 0;
        
    public:
        Control();
        ~Control(){};
        void SpeedCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
        void Pub();
        double ControlPid();
        
        
};

Control::Control(){

    control_data_pub = nh.advertise<std_msgs::Int16MultiArray>("/control_data", 1000);
    speed_sub = nh.subscribe("/ublox_gps/fix_velocity", 100, &Control::SpeedCallback, this);
    
    m_throttle = 0;
    m_steering = 195;
    
};

void Control::SpeedCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg){
    m_vehicle_speed = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)); // (m/s 단위)
}

void Control::Pub(){

    // control_data 배열 생성 및 초기화
    std_msgs::Int16MultiArray control_data;
    
    control_data.data.push_back(m_throttle); // 0번째 index = throttle
    control_data.data.push_back(m_steering); // 1번째 index = steering
    control_data_pub.publish(control_data); // control_data 배열 publish

}


double Control::ControlPid(){

  double setspeed = 0.5; //(m/s) 
  
  double error = setspeed - m_vehicle_speed;
  double previous_error = 0;

  std::cout<< " m_vehicle_speed(m/s) = " << m_vehicle_speed << std::endl;
  double k_p = Kp;
  double k_i = Ki;
  double k_d = Kd;

  double dt = 0.1;
  double intergral; 
  double differential;

  intergral = intergral + error * dt;
  differential = error - previous_error / dt;
  
  m_vehicle_speed = k_p * error + k_i * intergral - k_d * differential;
  
  if (m_vehicle_speed > 1){
      m_vehicle_speed = 1 ;
  } 
  previous_error = error;
  

  return m_vehicle_speed;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "control_node");
    Control control;
    ros::Rate loop_rate(50);
    
    while(ros::ok()){

        control.Pub();
        control.ControlPid();   
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}
