#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <algorithm>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <autonomous_msgs/CtrlCmd.h>
#include <autonomous_msgs/PoseMsg.h>
#include <autonomous_msgs/PolyfitLaneDataArray.h>

#define radian_to_degree (180/M_PI)
#define degree_to_radian (M_PI/180)



using namespace std;
using namespace Eigen;

class DrivingControl{

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_vehicle_input;
        ros::Subscriber lattice_sub;
        ros::Subscriber vehicle_status_sub; //from 로컬팀

        // autonomous_msgs::CtrlCmd drivinginput;
        std_msgs::Int32 drivinginput;
        autonomous_msgs::PolyfitLaneData polylane;
        autonomous_msgs::PoseMsg status_msg;
        
        
        nav_msgs::Path local_path;


        Matrix3d t;
        Matrix3d det_t;
        Matrix<double, 3, 1> global_path_point;
        Matrix<double, 3, 1> local_path_point;
        


        bool isodom = false;
        bool ispath = false;
        bool is_look_forward_point = false;

        double lfd_param = 0.0;
        double wheel_base = 0.0;
        double cur_steering = 0.0;

        int steering = 0;

        double position_x = 0.0;
        double position_y = 0.0;
        double heading = 0.0;
        double velocity = 0.0;

        //PID values
        double p_gain = 0.3;
        double i_gain = 0.0;
        double d_gain = 0.03;
        double prev_error = 0;
        double p_control = 0;
        double i_control = 0;
        double d_control = 0;
        double controlTime = 0.02;

        int index = 0;
        double theta = 0.0;
        



    public:
        //initialize
        DrivingControl(){

            // pub_vehicle_input = nh.advertise<autonomous_msgs::CtrlCmd>
            //                     ("/ctrl_cmd", 1);

            pub_vehicle_input = nh.advertise<std_msgs::Int32>
            ("/ctrl_cmd", 1);

            lattice_sub = nh.subscribe
                          ("/local_path", 10, &DrivingControl::path_callback, this);
            
            vehicle_status_sub = nh.subscribe
                                 ("/m_vehicle_pose", 10, &DrivingControl::vehiclestatus_callback, this);

            nh.param("auto_drive/lookahead", lfd_param, 4.0); //lookahead 포인트                            
        }

        ~DrivingControl(){}


        int mapValue(double value, double minFrom, double maxFrom, int minTo, int maxTo) {
            // 원래 범위에서의 비율 계산
            double ratio = (value - minFrom) / (maxFrom - minFrom);
            
            // 새로운 범위에서의 값 계산
            int result = static_cast<int>(minTo + ratio * (maxTo - minTo) + 0.5); // 반올림
            
            // 결과 값이 새로운 범위를 넘어가지 않도록 보정
            return std::min(maxTo, std::max(minTo, result));
        }
 

        void LateralControl(){
            if (ispath == true && isodom == true){
                is_look_forward_point = false;

                t << cos(heading), -sin(heading), position_x,
                    sin(heading), cos(heading), position_y,
                    0,          0,            1;
                
                //역행렬
                det_t << t(0,0), t(1,0), -(t(0,0)*position_x+t(1,0)*position_y),
                         t(0,1), t(1,1), -(t(0,1)*position_x+t(1,1)*position_y),
                         0,    0,      1;
                
                for(int i=0; i < local_path.poses.size();i++){
                    geometry_msgs::Point path_point;

                    //local path들을 path_point에 저장
                    path_point = local_path.poses[i].pose.position; 
                    index = local_path.poses[i].header.seq;
                    // std::cout<<"index"<<index<<std::endl;
                    // cout<<path_point<<endl;

                    //local path = global path point
                    global_path_point << path_point.x,
                                        path_point.y,
                                        1;
                    
                    //local path point는 차량 기준 좌표계로 변환
                    local_path_point =  det_t * global_path_point; //global, local 둘다 matrix
                    // lfd_param = (status_msg.velocity.x * 3.6 * 0.237) + 5.29;

                    // cout<<local_path_point(0,0)<<endl;


                    if (local_path_point(0,0)>0){
                        double dis = sqrt(pow(local_path_point(0,0),2)+pow(local_path_point(1,0),2));//문제 해결
                        // cout<<dis<<endl; // 주석 처리함!!!!!!!!!!!! 언수
                        if (dis >= lfd_param){
                            // foward_point = path_point;
                            is_look_forward_point = true;
                            break;
                        }
                    }
                    else{
                        // cout<<"far"<<endl;
                    }
                }

                theta=atan2(local_path_point(1,0),local_path_point(0,0));
                if (is_look_forward_point){
                    
                    // cur_steering = atan2(2*2.7*sin(theta)/lfd_param,1.0);
                    cur_steering = theta;//수정
                    // std::cout<<steering*180.0/M_PI<<std::endl;
                }
                else{
                    cur_steering = 0.0;
                    // std::cout<<"여기"<<std::endl;
                }

                double steering_in = -cur_steering * radian_to_degree;

                if(steering_in > 30){
                    steering_in = 30;
                }
                else if(steering_in < -30){
                    steering_in = -30;
                }

                steering = mapValue(steering_in, -30, 30, 128, 255);

            }
        }

        

        ///속도출력 -> curvedbased velocity 바꿔야될수도 있음
        // void VelocityControl(){
        //     // double pid_output = 0;
            
        //     if (abs(steering*180.0/M_PI)>5){
        //         Velocity(10);
        //     }
        //     else if (abs(steering*180.0/M_PI)>15){
        //         Velocity(8);
        //     }
        //     else{
        //         Velocity(10);
        //     }
            
            
        // }

        // void velocity(double target_vel){
        //     double error = target_vel - velocity*3.6; //3.6은 속도 단위보정  단위 맞게 바꿔줘야함

        //     p_control = p_gain * error;
        //     i_control += i_gain * error * controlTime;
        //     d_control = d_gain * (error-prev_error) / controlTime;

        //     double output = p_control + i_control + d_control;
        //     prev_error = error;

        //     if(output > 0){
        //         accel = output;
        //         brake = 0;
        //     }
        //     else{
        //         accel = 0;
        //         brake = -output;
        //     }
        //     pub(); 
            
        // }


        void pub()
        {
            // drivinginput.steering = cur_steering;
            // drivinginput.accel = 196;
            // drivinginput.brake = 0;
            // //float64 타입
            // drivinginput.steering = steering;
            drivinginput.data = steering;
            ROS_INFO("Steering : %d", drivinginput.data);
            // drivinginput[1] = 196;
            pub_vehicle_input.publish(drivinginput);
        }

    private:

        void vehiclestatus_callback(const autonomous_msgs::PoseMsg::ConstPtr &msg){
            if(!isodom){
                isodom = true;
            }
            status_msg = *msg;

            position_x = status_msg.x;
            position_y = status_msg.y;
            heading = status_msg.heading;
            // velocity = status_msg.velocity;
        }

        void path_callback(const nav_msgs::Path::ConstPtr &msg){
            if(!ispath){
                ispath = true;
            }
            local_path = *msg;
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "auto_drive");
    DrivingControl dctrl;

    ros::Rate loop_rate(80);
    while(ros::ok()){
        dctrl.LateralControl();
        dctrl.pub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}