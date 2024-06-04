#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <eigen3/Eigen/Dense>
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



using namespace std;

class DrivingControl{

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_vehicle_input;
        ros::Subscriber lattice_sub;
        ros::Subscriber vehicle_status_sub; //from 로컬팀

        autonomous_msgs::CtrlCmd drivinginput;
        autonomous_msgs::PolyfitLaneData polylane;
        autonomous_msgs::PoseMsg status_msg;
        
        
        nav_msgs::Path local_path;
        


        bool isodom = false;

        double lfd_param = 0.0;
        double wheel_base = 0.0;
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

            pub_vehicle_input = nh.advertise<autonomous_msgs::CtrlCmd>
                                ("/ctrl_cmd", 1);

            lattice_sub = nh.subscribe
                          ("/local_path", 10, &DrivingControl::path_callback, this);
            
            vehicle_status_sub = nh.subscribe
                                 ("/m_vehicle_pose", 10, &DrivingControl::vehiclestatus_callback, this);

            nh.param("auto_drive/lookahead", lfd_param, 5.0); //lookahead 포인트                            
        }

        ~DrivingControl(){}
 
        void PolyfitLane(){
            // nav_msgs::Path path_lane;
            // path_lane.header.frame_id = "/body";
            // path_lane.clear();

            // for(auto poses = 0 ; local_path.poses.size()>poses ; ++poses){
                
            // } poses = point ,local_path = lane
            polylane.frame_id = "/body";
            // polylane.polyfitLanes.clear();

            //////n차 피팅/////
            Eigen::MatrixXf x_points(local_path.poses.size(), 3);
            Eigen::VectorXf y_points(local_path.poses.size());

            for(auto i_point = 0 ; local_path.poses.size()>i_point ; ++i_point){
                x_points(i_point,0) = 1;
                x_points(i_point,1) = local_path.poses[i_point].pose.position.x;
                x_points(i_point,2) = (local_path.poses[i_point].pose.position.x,2);
                // x_points(i_point,2) = (local_path.poses[i_point].pose.position.x,3);//3차피팅

                y_points(i_point) = local_path.poses[i_point].pose.position.y;
            }

            /////곡선의 계수 생성/////
            Eigen::VectorXf a_Vector(3);
            a_Vector = (x_points.transpose() * x_points).inverse() * x_points.transpose() * y_points;
            
            polylane.frame_id = "/body";
            
            polylane.a0 = a_Vector(0);
            polylane.a1 = a_Vector(1);
            polylane.a2 = a_Vector(2);
            polylane.a3 = 0;

            cout << "a0 : " << polylane.a0 << endl;
            cout << "a1 : " << polylane.a1 << endl;
            cout << "a2 : " << polylane.a2 << endl;
            cout << "a3 : " << polylane.a3 << endl;
        }

        void LateralControl(){
            double gx = lfd_param;
            double gy = polylane.a0 
                        + polylane.a1 * lfd_param 
                        + polylane.a2 * lfd_param * lfd_param
                        + polylane.a3 * lfd_param * lfd_param * lfd_param;
            double ld = sqrt(pow(gx,2)+pow(gy,2));
            // cout << "debugging : " << gy << endl;
            double e = gy;
            cur_steering = atan2(2*wheel_base*e,pow(ld,2));
            
            // drivinginput.steering = cur_steering;
            // prev_steering = cur_steering;
            
            // cout<<"steering : "<<cur_steering<<endl;
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
            drivinginput.steering = cur_steering;
            // drivingInput.accel = accel;
            // drivingInput.brake = brake;
            //float64 타입
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
            // if(!ispath){
            //     ispath = true;
            // }
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