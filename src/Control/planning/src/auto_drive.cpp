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
        ros::Publisher lfd_pub;
        ros::Publisher car_path_pub;
        ros::Subscriber lattice_sub;
        ros::Subscriber vehicle_status_sub; //from 로컬팀
        

        // autonomous_msgs::CtrlCmd drivinginput;
        std_msgs::Int32 drivinginput;
        autonomous_msgs::PolyfitLaneData polylane;
        autonomous_msgs::PoseMsg status_msg;
        
        
        nav_msgs::Path local_path;
        


        bool isodom = false;
        bool ispath = false;
        bool is_look_forward_point = false;

        bool isvel = false;

        //전역 좌표의 차량 기준 좌표로의 변환
        Matrix3d t;
        Matrix3d det_t;
        Matrix<double, 3, 1> global_path_point;
        Matrix<double, 3, 1> car_point;

        nav_msgs::Path car_path;//차량기준 좌표계로 변환한 local_path(최종 저장 지점)

        double lfd_param = 0.0;
        double wheel_base = 0.375;
        double cur_steering = 0.0;

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

        



    public:
        //initialize
        DrivingControl(){

            // pub_vehicle_input = nh.advertise<autonomous_msgs::CtrlCmd>
            //                     ("/ctrl_cmd", 1);

            pub_vehicle_input = nh.advertise<std_msgs::Int32>
                                ("/ctrl_cmd", 1);

            lfd_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/lfd_point", 1);
            car_path_pub = nh.advertise<nav_msgs::Path>
                                ("/car_path", 1);


            lattice_sub = nh.subscribe
                          ("/local_path", 10, &DrivingControl::path_callback, this);
            
            vehicle_status_sub = nh.subscribe
                                 ("/m_vehicle_pose", 10, &DrivingControl::vehiclestatus_callback, this);

            nh.param("auto_drive/lookahead", lfd_param, 1.2); //lookahead 포인트                      
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

        void CarPathMaker(){

            car_path.header.frame_id = "/map";
            car_path.poses.clear();
            ////////차량 원점 좌표로 변환////////
            is_look_forward_point = false;

            
            //heading은 라디안...
            t << cos(heading), -sin(heading), position_x,
                    sin(heading), cos(heading), position_y,
                    0,          0,            1;
                
            //역행렬
            det_t << t(0,0), t(1,0), -(t(0,0)*position_x+t(1,0)*position_y),
                     t(0,1), t(1,1), -(t(0,1)*position_x+t(1,1)*position_y),
                          0,      0,                                      1;

            for(int i=0; i < local_path.poses.size();i++){
                geometry_msgs::Point path_point; //임시로 localpoint 점을 받아옴
                // geometry_msgs::PoseStamped path_point;
                geometry_msgs::PoseStamped car_path_point; //변환된 경로 점을 임시로 저장
                // car_path_point.pose.clear();

                

                ///////////////////////////////////
                path_point = local_path.poses[i].pose.position;
                int index = local_path.poses[i].header.seq; //index 확인용
                //추가로 인덱스 경로에 저장하려면 car_path_point에 저장
                ///////////////////////////////////

                //local path들을 path_point에 저장
                // path_point = local_path.poses[i].pose.position; 
                // index = local_path.poses[i].header.seq;


                // std::cout<<"index"<<index<<std::endl;
                // cout<<path_point<<endl;

                //local path = global path point
                global_path_point << path_point.x,
                                    path_point.y,
                                    1;
                
                //local path point는 차량 기준 좌표계로 변환
                car_point =  det_t * global_path_point; //global, local 둘다 matrix

                //x좌표
                car_path_point.pose.position.x = car_point(0,0);
                car_path_point.pose.position.y = car_point(1,0);
                // cout<<car_point<<endl;

                car_path.poses.push_back(car_path_point);
                car_path_pub.publish(car_path);
            }
        }
 
        void PolyfitLane(){
            // nav_msgs::Path path_lane;
            // path_lane.header.frame_id = "/body";
            // path_lane.clear();

            // for(auto poses = 0 ; local_path.poses.size()>poses ; ++poses){
                
            // } poses = point ,local_path = lane
            // polylane.frame_id = "world";
            // polylane.polyfitLanes.clear();

            //필요시 ispath, isodom if로 삽입



            
             

                // cout<<local_path_point(0,0)<<endl;






            CarPathMaker();//경로 변환

            //////n차 피팅/////
            Eigen::MatrixXf x_points(car_path.poses.size(), 3);
            Eigen::VectorXf y_points(car_path.poses.size());

            for(auto i_point = 0 ; car_path.poses.size()>i_point ; ++i_point){
                x_points(i_point,0) = 1;
                x_points(i_point,1) = car_path.poses[i_point].pose.position.x;
                x_points(i_point,2) = pow(car_path.poses[i_point].pose.position.x,2);
                // x_points(i_point,2) = (local_path.poses[i_point].pose.position.x,3);//3차피팅

                y_points(i_point) = car_path.poses[i_point].pose.position.y;
            }

            // cout<<x_points<<endl;

            /////곡선의 계수 생성/////
            Eigen::VectorXf a_Vector(3);
            a_Vector = (x_points.transpose() * x_points).inverse() * x_points.transpose() * y_points;
            
            polylane.frame_id = "/map";
            
            polylane.a0 = a_Vector(0);
            polylane.a1 = a_Vector(1);
            polylane.a2 = a_Vector(2);
            polylane.a3 = 0;
            // cout<<polylane<<endl;
        }

        void LateralControl(){
            double gx = lfd_param;
            double gy = polylane.a0 
                        + polylane.a1 * lfd_param 
                        + polylane.a2 * lfd_param * lfd_param
                        + polylane.a3 * lfd_param * lfd_param * lfd_param;


            Matrix<double, 3, 1> lfdp;
            lfdp << gx,
                    gy,
                     1;
            Matrix<double, 3, 1> g_lfd;
            g_lfd = t * lfdp;

            // lfd 찍히는 부분            
            geometry_msgs::PoseStamped lookahead_point;
            lookahead_point.header.frame_id = "world";
            lookahead_point.pose.position.x = g_lfd(0,0);
            lookahead_point.pose.position.y = g_lfd(1,0);

            lfd_pub.publish(lookahead_point);


            double ld = sqrt(pow(gx,2)+pow(gy,2));
            cout<<"ld: "<<ld<<endl;
            double e = gy;
            // cout<<e<<endl;
            // cur_steering = atan2(2*wheel_base*e,pow(ld,2));
            cur_steering = atan2(2*wheel_base*e,ld);
            
            double steering_in = - cur_steering*radian_to_degree;

            if(steering_in > 30){
                    steering_in = 30;
                }
            else if(steering_in < -30){
                steering_in = -30;
            }

            // drivinginput.steering = mapValue(steering_in, -30, 30, 128, 255);
            drivinginput.data = mapValue(steering_in, -30, 30, 128, 255);

            // prev_steering = cur_steering;
            
            // cout<<"steering : "<<drivinginput.steering<<endl;
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
            // drivinginput.steering = cur_steering * radian_to_degree;
            // drivingInput.accel = accel;
            // drivingInput.brake = brake;
            //float64 타입
            if(isvel == false){
                drivinginput.data = 195;
            }
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
            heading = status_msg.heading * degree_to_radian;
            isvel = status_msg.isenable;
        
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

    ros::Rate loop_rate(50);
    while(ros::ok()){
        dctrl.PolyfitLane();
        dctrl.LateralControl();
        dctrl.pub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}