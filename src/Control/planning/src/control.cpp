#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <autonomous_msgs/CtrlCmd.h>
#include <string>
#include <thread>
#include <cmath>

using namespace std;


class Control{
    private:
        ros::NodeHandle nh;
        ros::Publisher control_data_pub;

        autonomous_msgs::CtrlCmd drivinginput;

        int m_throttle;
        int m_steering;
        
    public:
        Control();
        ~Control(){};
        void Pub();
        
};

Control::Control(){

    control_data_pub = nh.advertise<std_msgs::Int16MultiArray>("/control_data", 1000);
    
};

int mapValue(double value, double minFrom, double maxFrom, int minTo, int maxTo) {
    // 원래 범위에서의 비율 계산
    double ratio = (value - minFrom) / (maxFrom - minFrom);
    
    // 새로운 범위에서의 값 계산
    int result = static_cast<int>(minTo + ratio * (maxTo - minTo) + 0.5); // 반올림
    
    // 결과 값이 새로운 범위를 넘어가지 않도록 보정
    return std::min(maxTo, std::max(minTo, result));
}

void Control::Pub(){

    m_throttle = driving;
    m_steering = 140;

    double steering_in = drivinginput.steering;

    if(steering_in > 30){
        steering_in = 30;
    }
    else if(steering_in < -30){
        steering_in = -30;
    }

    int steering = mapValue(steering_in, -30, 30, 128, 255);
    cout<<steering<<endl;

    // control_data 배열 생성 및 초기화
    std_msgs::Int16MultiArray control_data;

    
    control_data.data.push_back(steering);
    // control_data.data.push_back(m_throttle); // 0번째 index = throttle
    // control_data.data.push_back(m_steering); // 1번째 index = steering
    control_data_pub.publish(control_data); // control_data 배열 publish

}

void control_callback(const autonomous_msgs::CtrlCmd::ConstPtr &msg){
            // if(!ispath){
            //     ispath = true;
            // }
            drivinginput = *msg;
        }


int main(int argc, char **argv){

    ros::init(argc, argv, "control_node");
    Control control;
    ros::Rate loop_rate(50);
    
    while(ros::ok()){

        control.Pub();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;   
}
