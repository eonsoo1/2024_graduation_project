#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <string>
#include <thread>
#include <cmath>

using namespace std;


class Control{
    private:
        ros::NodeHandle nh;
        ros::Publisher control_data_pub;

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

void Control::Pub(){

    m_throttle = 196;
    m_steering = 140;

    // control_data 배열 생성 및 초기화
    std_msgs::Int16MultiArray control_data;

    
    
    control_data.data.push_back(m_throttle); // 0번째 index = throttle
    control_data.data.push_back(m_steering); // 1번째 index = steering
    control_data_pub.publish(control_data); // control_data 배열 publish

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
