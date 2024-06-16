#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <numeric>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <limits>
#include <array>


#include <autonomous_msgs/ObstacleArray.h>
#include <autonomous_msgs/PoseMsg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;
using namespace Eigen;

#define PI std::acos(-1)
#define radian_to_degree (180/PI)
#define degree_to_radian (PI/180)



class pathreader{
    protected:
        ros::NodeHandle nh;
        ros::Publisher path_pub;
        ros::Publisher local_pub;
        ros::Publisher lattice_pub;
        ros::Subscriber vehicle_status_sub;
        ros::Subscriber object_status_sub;

        ros::Publisher vis_pub;

        vector<ros::Publisher> publishers;

        //subscribe msg
        autonomous_msgs::PoseMsg status_msg;
        autonomous_msgs::ObstacleArray lidar_object_data;
        autonomous_msgs::ObstacleArray object_data;

        autonomous_msgs::ObstacleArray walls;


        //variable
        string file_path;
        string path_folder_name;
        nav_msgs::Path global_path;
        nav_msgs::Path local_path;
        nav_msgs::Path lattice_path;
        // autonomous_msgs::LatticePathData lattice_lane;

        //lattice 경로저장 벡터
        vector<nav_msgs::Path> lat_path;

        visualization_msgs::MarkerArray zone;


        Matrix3d trans_matrix;
        Matrix3d det_trans_matrix;

        double obj_size = 0.0;

        double wheel_base = 0.375;


        int prev_waypoint = 0;
        int index = 0;

        double gps_offset = 0.0;

        //check object
        bool is_crash = false;
        //collision check
        int selected_lane = -1;
    
    public:
        pathreader(){
            file_path = ros::package::getPath("planning");

            path_pub = nh.advertise<nav_msgs::Path>
                       ("/global_path", 1);
            local_pub = nh.advertise<nav_msgs::Path>
                       ("/local_path", 1);
            
            vis_pub = nh.advertise<visualization_msgs::MarkerArray>
                       ("wall_marker", 0 );

            lattice_pub = nh.advertise<nav_msgs::Path>
                       ("/lattice_path", 1);

            vehicle_status_sub = nh.subscribe
                                ("/m_vehicle_pose", 10, &pathreader::status_callback, this);

            object_status_sub = nh.subscribe
                                ("/object", 10, &pathreader::object_callback, this);

        }
        ~pathreader(){}

        void pub(const nav_msgs::Path &path){
            path_pub.publish(path);
        }

        void status_callback(const autonomous_msgs::PoseMsg::ConstPtr &msg){
            status_msg = *msg;
            status_msg.heading = status_msg.heading * degree_to_radian;
        }

        void object_callback(const autonomous_msgs::ObstacleArray::ConstPtr &msg){
            // lidar_object_data = *msg;
            // objectTranslation();
            // buildSafetyObject();

            // buildWall();
            object_data = *msg;
            // buildWall();
            //벽 세워보자
            
            

        }

        void buildWall(){
            // 장애물의 바깥쪽을 제한하는 방법으로 벽세우기
            for(int i = 0 ; object_data.cluster.size()>i ; ++i){
                autonomous_msgs::Obstacle wall;
                wall.mid_x = object_data.cluster[i].mid_x + 0.5;
                wall.mid_y = object_data.cluster[i].mid_y - 0.5;
                wall.width = 0.1;
                wall.height = 0.1;
                object_data.cluster.push_back(wall);
            }




            // //local을 차량좌표로 변환한 거에서 벽세워야
            // for(int i = 0 ; local_path.poses.size()>i ; ++i){
            //     autonomous_msgs::Obstacle wall;
            //     wall.mid_x = local_path.poses[i].pose.position.x + 0.5;
            //     wall.mid_y = local_path.poses[i].pose.position.y + 0.5;
            //     wall.width = 0.1;
            //     wall.height = 0.1;
            //     object_data.cluster.push_back(wall);
            //     visualization_msgs::Marker wall_marker;
            //     wall_marker.header.frame_id = "/map";
            //     wall_marker.pose.position.x = wall.mid_x;
            //     wall_marker.pose.position.x = wall.mid_y;
            //     wall_marker.pose.position.z = 0.0;
            //     wall_marker.pose.orientation.x = 0.0;
            //     wall_marker.pose.orientation.y = 0.0;
            //     wall_marker.pose.orientation.z = 0.0;
            //     wall_marker.pose.orientation.w = 1.0;
            //     wall_marker.scale.x = 0.1;
            //     wall_marker.scale.y = 0.1;
            //     wall_marker.scale.z = 0.1;
            //     wall_marker.color.a = 1.0; // Don't forget to set the alpha!
            //     wall_marker.color.r = 0.0;
            //     wall_marker.color.g = 1.0;
            //     wall_marker.color.b = 0.0;
            //     zone.markers.push_back(wall_marker);

            // }

            //     cout<<object_data<<endl;

                
            //     vis_pub.publish(zone);

            
        }

        // void objectDelete(){
        //     for(int obstacle = 0 ; object_data.cluster.size() > obstacle ; ++obstacle){
        //         double dx = object_data.cluster[obstacle].mid_x - status_msg.x;
        //         double dy = object_data.cluster[obstacle].mid_y - status_msg.y;
        //         double distance = sqrt(pow(dx,2)+pow(dy,2));

        //     }
        // }

        
        void buildSafetyObject(){
            // autonomous_msgs::Obstacle data;
            autonomous_msgs::ObstacleArray safety_objects;
            for(int obstacle = 0 ; lidar_object_data.cluster.size() > obstacle ; ++obstacle){
                double width = lidar_object_data.cluster[obstacle].width;
                double height = lidar_object_data.cluster[obstacle].height;
                ////////////////////////////////////////////////////////////////
                double tuner = 0.6; // 장애물이 인식되는 크기가 일정 수준 이하일 경우의 threshold 설정
                //tuner를 밑에 코드가 radius가 0.63임을 고려하여 경로가 튀지 않도록 결정해주어야함

                if(sqrt(pow(width,2) + pow(height,2)) < tuner){
                    autonomous_msgs::Obstacle safety_object;
                    safety_object.mid_x = lidar_object_data.cluster[obstacle].mid_x + 0.55;
                    safety_object.mid_y = lidar_object_data.cluster[obstacle].mid_y;
                    safety_object.width = 0.45;
                    safety_object.height = 0.45;
                    safety_objects.cluster.push_back(safety_object);
                }
                ////////////////////////////////////////////////////////////////
            }

            for(int i = 0 ; safety_objects.cluster.size()>i; ++i){
                lidar_object_data.cluster.push_back(safety_objects.cluster[i]);
            }
            
        }


        void objectTranslation(){
            Matrix3d t;
            Matrix3d det_t;

            autonomous_msgs::Obstacle data;

            Matrix<double, 3, 1> global_obj_point;
            Matrix<double, 3, 1> lidar_obj_point;

            double position_x = status_msg.x;
            double position_y = status_msg.y;
            double heading = status_msg.heading;

            //heading은 라디안...
            t << cos(heading), -sin(heading), position_x,
                    sin(heading), cos(heading), position_y,
                    0,          0,            1;
                
            //역행렬
            det_t << t(0,0), t(1,0), -(t(0,0)*position_x+t(1,0)*position_y),
                     t(0,1), t(1,1), -(t(0,1)*position_x+t(1,1)*position_y),
                          0,      0,                                      1;

            for(int obstacle = 0 ; lidar_object_data.cluster.size() > obstacle ; ++obstacle){
                lidar_obj_point << lidar_object_data.cluster[obstacle].mid_x + gps_offset,
                                   lidar_object_data.cluster[obstacle].mid_y,
                                                                     1;

                global_obj_point = t * lidar_obj_point;
                data.mid_x = global_obj_point(0,0);
                data.mid_y = global_obj_point(1,0);
                data.width = lidar_object_data.cluster[obstacle].width;
                data.height = lidar_object_data.cluster[obstacle].height;

                object_data.cluster.push_back(data);

            }


        }

        //충돌여부 판단하여 경로 생성하는 시점
        // 거리값 이상함
        bool checkObject(){
            is_crash = false;
            double dis = 0.0;
            double check_dis = 2.0;

            for(int obstacle = 0 ; object_data.cluster.size() > obstacle ; ++obstacle){
                double width = object_data.cluster[obstacle].width;
                double height = object_data.cluster[obstacle].height;
                double radius = sqrt(pow(width,2) + pow(height,2));
                check_dis = radius / 2;
                // cout<<check_dis<<endl;
                //local_path.poses.size()- 4  : 6번째 점까지 탐색
                for(int path = 0 ; local_path.poses.size()- 4 > path ; ++path){
                    dis = sqrt(pow(local_path.poses[path].pose.position.x 
                               - object_data.cluster[obstacle].mid_x, 2)
                             + pow(local_path.poses[path].pose.position.y 
                               - object_data.cluster[obstacle].mid_y, 2));

                    // cout<<dis<<endl;
                    // 충돌 감지 거리
                    if(dis < check_dis){
                        is_crash = true;
                        break;
                    }
                }
            }
            // cout<<is_crash<<endl;
            return is_crash;
        }


        // 거리 체크를 어떤 점을 기준으로 할 지 정해야 함
        void collisionCheck(const vector<nav_msgs::Path>& lattice_lane){

            selected_lane = -1;

            array<double, 8> lane_weight = {4, 3, 2, 1, 1, 2, 3, 4};
            // array<double, 8> lane_weight = {8, 7, 6, 5, 1, 2, 3, 4};
            double dis = 0.0;
            double check_dis = 1.0;


            for(int obstacle = 0 ; object_data.cluster.size() > obstacle ; ++obstacle){
                double width = object_data.cluster[obstacle].width;
                double height = object_data.cluster[obstacle].height;
                double radius = sqrt(pow(width,2) + pow(height,2));
                check_dis = radius / 2; //여기서 체크는 차량 크기정도만 해도 될듯 장애물과 경로사이의 거리임
                for(int lane = 0 ; lattice_lane.size() > lane ; ++lane){
                    for(int pos = 0 ; lattice_lane[lane].poses.size() > pos ; ++pos){
                        dis = sqrt(pow(object_data.cluster[obstacle].mid_x
                                        - lattice_lane[lane].poses[pos].pose.position.x,2)
                                 + pow(object_data.cluster[obstacle].mid_y
                                        - lattice_lane[lane].poses[pos].pose.position.y,2));
                        
                        if(dis < check_dis){
                            lane_weight[lane] = lane_weight[lane] + 100;
                        }
                    }
                }
            }
            for(int lane = 0 ; lane_weight.size()>lane ; ++lane){
                cout<<"plane "<<lane<<"("<<lane_weight[lane]<<")";
            }
            cout<<""<<endl;

            lane_weight[0] = (lane_weight[0]+lane_weight[1]) / 2;
            lane_weight[1] = (lane_weight[1]+lane_weight[2]) / 2;
            lane_weight[2] = (lane_weight[2]+lane_weight[3]) / 2;

            lane_weight[7] = (lane_weight[6]+lane_weight[7]) / 2;
            lane_weight[6] = (lane_weight[5]+lane_weight[6]) / 2;
            lane_weight[5] = (lane_weight[4]+lane_weight[5]) / 2;

            ///////////////////////////////////////////////////////////////////
            // double right = lane_weight[0] + lane_weight[1] + lane_weight[2];
            // double left = lane_weight[7] + lane_weight[6] + lane_weight[5];
            // vector<double> minindex;
            // if(right >= left){
            //     //좌측으로
                
            //     auto minIndex = min_element(lane_weight.begin() + 3, lane_weight.end());
            //     selected_lane = *minIndex; //lane index 반환됨
            // }
            // else if(right < left){
            //     //우측으로
            //     auto minIndex = min_element(lane_weight.begin(), lane_weight.end() - 3);
            //     selected_lane = *minIndex;; //lane index 반환됨
            // }
            ///////////////////////////////////////////////////////////////////
            
            for(int lane = 0 ; lane_weight.size()>lane ; ++lane){
                cout<<"lane "<<lane<<"("<<lane_weight[lane]<<")";
            }
            cout<<""<<endl;
            
            vector<int> minindex;
            auto minIndex = min_element(lane_weight.begin(), lane_weight.end());
            // selected_lane = distance(lane_weight.begin(), minIndex); //lane index 반환됨
            double min_weight = *minIndex;

            minindex.clear();
            for(int i = 0; lane_weight.size()>i ; ++i){
                if(lane_weight[i] == min_weight){
                    minindex.push_back(i);
                }
            }

            auto max_lane = max_element(minindex.begin(), minindex.end());
            selected_lane = *max_lane; // 웨이트 가장 작은 것들 중 레인값 가장 높은 것




            // 인덱스가 작은 쪽으로 이동
            cout<<"min :"<<minIndex<<endl;
            cout<<"sel :"<<selected_lane<<endl;
            // return selected_lane;

        }

        // Helper function to calculate the binomial coefficient
        int binomialCoeff(int n, int k) {
            if (k == 0 || k == n) return 1;
            return binomialCoeff(n-1, k-1) + binomialCoeff(n-1, k);
        }

        // Calculate Bernstein polynomial value for given i, n, and t
        // i는 다항식의 차수, n은 다항식의 총 차수, t는 매개변수
        double bernstein(int i, int n, double t) {
            return binomialCoeff(n, i) * pow(t, i) * pow(1 - t, n - i);
        }


        void latticePlanner(){

            // 생성되는 lattice 경로들을 저장하는 벡터
            // vector<nav_msgs::Path> out_path;
            
            // 경로 생성
            double pose_x = status_msg.x - 0.055; //1.26은 차량 중심으로 보정해준 것
            double pose_y = status_msg.y;
            // double velocity = status_msg.velocity * 3.6;
            double velocity = 0.4 * 3.6;

            // 경로를 생성할 거리
            int look_distance = static_cast<int>(velocity * 0.2 * 2);
            

            if(look_distance < 8){
                look_distance = 8;
            }
            // cout<<"//////////////////"<<endl;
            // cout<<local_path.poses.size()<<endl;
            // cout<<look_distance<<endl;
            // local path의 길이가 look distance보다 길어야 경로 생성 가능
            if(local_path.poses.size() > look_distance){
                // cout<<"here"<<endl;
                geometry_msgs::Point global_ref_start_point;
                geometry_msgs::Point global_ref_next_point;
                geometry_msgs::Point global_ref_end_point;

                lat_path.clear();

                global_ref_start_point = local_path.poses[0].pose.position;
                global_ref_next_point = local_path.poses[1].pose.position;
                global_ref_end_point = local_path.poses[6].pose.position;
                //원래 global_ref_end_point = local_path.poses[look_distance * 2].pose.position;

                //end point에 맞는 heading으로 끝점 좌표축 변환//////////////////////////////////////////////////////////////////////

                cout<<global_ref_end_point<<endl;

                double theta = atan2(global_ref_next_point.y - global_ref_start_point.y, global_ref_next_point.x - global_ref_start_point.x);

                Vector2d translation(global_ref_start_point.x, global_ref_start_point.y);

                // 변환행렬 translation point를 global_ref_start_point로 사용
                trans_matrix << cos(theta), -sin(theta), translation(0),
                                sin(theta),  cos(theta), translation(1),
                                         0,           0,              1;

                det_trans_matrix << trans_matrix(0,0), trans_matrix(1,0), -(trans_matrix(0,0) * translation(0) + trans_matrix(1,0) * translation(1)),
                                    trans_matrix(0,1), trans_matrix(1,1), -(trans_matrix(0,1) * translation(0) + trans_matrix(1,1) * translation(1)),
                                                    0,                 0,                                                                          1;

                Matrix<double, 3, 1> world_end;
                world_end << global_ref_end_point.x,
                             global_ref_end_point.y,
                                                  1;
                // cout<<world_end<<endl;

                // local path의 끝점들을 차량 기준 좌표계로 나타낸 것
                Matrix<double, 3, 1>  local_end;
                local_end = det_trans_matrix * world_end;
                // cout<<local_end<<endl;

                // 전역 좌표계 기준의 차량 좌표
                Matrix<double, 3, 1> world_vehicle_position;
                world_vehicle_position << pose_x,
                                          pose_y,
                                               1; 
                
                Matrix<double, 3, 1> local_vehicle_position; //local path의 시작점을 원점으로 하는 차량좌표
                local_vehicle_position = det_trans_matrix * world_vehicle_position;

                // cout<<local_vehicle_position<<endl;


                // vector<double> lane_off_set = {-4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 4.0};
                vector<double> lane_off_set = {-2.0, -1.5, -1.0, -0.5, 0.5, 1.0, 1.5, 2.0};
                // vector<double> lane_off_set = {-0.8, -0.6, -0.4, -0.2, 0.5, 1.0, 1.5, 2.0};


                MatrixXd lattice_end_points(lane_off_set.size(), 2);

                // lattice 경로 끝점들 생성
                for(auto lane = 0 ; lane_off_set.size() > lane ; ++lane){
                    lattice_end_points(lane,0) = local_end(0,0);
                    lattice_end_points(lane,1) = local_end(1,0) + lane_off_set[lane];
                    // cout<<"/////////////////////////"<<endl;
                    // cout<<lattice_end_points(lane,1)<<endl;
                }

                // cout<<lattice_end_points.rows()<<endl;
                
                // end_point는 벡터
                for(int row = 0 ; lattice_end_points.rows()>row ; ++row){
                    nav_msgs::Path lattice_path;
                    lattice_path.header.frame_id = "/map";

                    double x_interval = 0.1;

                    vector<double> x;
                    vector<double> y;

                    double xs = 0; //x0
                    double xf = lattice_end_points(row, 0); //x1
                    // cout<<xf<<endl;

                    double ps = local_vehicle_position(1,0); //y0
                    double pf = lattice_end_points(row, 1); //y1
                    // cout<<pf<<endl;

                    double xm = (xs + xf) / 2;
                    double pm = (ps + pf) / 2;

                    //qubic spline 생성에 사용할 점 개수
                    double dx_num = xf / x_interval;
                    int x_num = static_cast<int>(dx_num);
                    // cout<<x_num<<endl;

                    // 점 개수에 알맞은 x 좌표 먼저 생성
                    // for(int i = 0 ; x_num > i ; ++i){
                    //     x.push_back(i*x_interval);
                    //     // cout<<i<<" : "<<i*x_interval<<endl;
                    // }
                    // cout<<"/////////////////////////"<<endl;
                    // cout<<x<<endl;
                    ///////////////////////Bézier Curve //////////////////////////////
                    // Vector2d p0(xs,ps);
                    // Vector2d p1(xm,pm);
                    // Vector2d p2(xf,pf);

                    Vector2d p0(xs,ps);
                    Vector2d p1(xs + ((xs+xm)/2),ps);
                    Vector2d p2(xm - ((xs+xm)/2),pf);
                    Vector2d p3(xf,pf);

                    for(int p = 0 ; dx_num > p ; ++p){
                        double t = static_cast<double>(p) / dx_num; //t를 증가시켜가며 곡선 생성

                        // Vector2d dP = 2 * (1 - t) * (p1 - p0) + 2 * t * (p2 - p1);

                        // double curvature = abs(dP(1) * (p2(0) - p0(0)) - dP(0) * (p2(1) - p0(1))) / pow((dP(0) * dP(0) + dP(1) * dP(1)), 1.5);

                        Vector2d d1 = 3 * pow(1 - t, 2) * (p1 - p0) + 6 * (1 - t) * t * (p2 - p1) + 3 * pow(t, 2) * (p3 - p2);
                        Vector2d d2 = 6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);
                        double curvature = d1(0) * d2(1) - d1(1) * d2(0);
                        curvature /= pow(d1.norm(), 3);

                        // Check if curvature exceeds the maximum curvature
                        // max_steering = 30deg
                        // 최소회전반경 = L(wheel_base)/tan(max_steering)
                        // max_curvature = 1/최소회전반경
                        double max_steering = 30 * degree_to_radian;
                        double min_radius = wheel_base / tan(max_steering);
                        double k_max = 1 / min_radius;
                        bool curvature_exec = false;
                        // cout<<"curvature : "<<curvature<<endl;
                        // cout<<"k_max : "<<k_max<<endl;

                        // if (abs(curvature) > k_max) {
                        //     cout << "Curvature exceeds maximum curvature at t = " << t << endl;
                        //     break;
                        // }
                        // if(curvature_exec) {
                        //     p1(1) = (p0(1) + p3(1)) / 2.0;
                        //     p2(1) = (p0(1) + p3(1)) / 2.0;
                        // }

                        // Calculate the point on the Bézier curve using Bernstein polynomials
                        Vector2d point = bernstein(0, 3, t) * p0 + bernstein(1, 3, t) * p1 + bernstein(2, 3, t) * p2 + bernstein(3, 3, t) * p3;
                        x.push_back(point(0));
                        y.push_back(point(1));
                    }










                    //////////////////////////////////////////////////////////////////

                    // Eigen::Matrix4d A;
                    // Eigen::Vector4d B;

                    // A << std::pow(xs, 3), std::pow(xs, 2), xs, 1,
                    //      std::pow(xf, 3), std::pow(xf, 2), xf, 1,
                    //      3 * std::pow(xs, 2), 2 * xs, 1, 0,
                    //      3 * std::pow(xf, 2), 2 * xf, 1, 0;

                    // B << ps, pf, 0, 0;

                    //qubic spline 생성
                    // vector<double> a (4,0.0);
                    // a[0] = ps;
                    // a[1] = 0;
                    // a[2] = 3.0 * (pf - ps) / (xf * xf);
                    // a[3] = -2.0 * (pf - ps) / (xf * xf * xf);

                    // Vector4d coeffs = A.colPivHouseholderQr().solve(B);
                    // a[0] = coeffs(0);
                    // a[1] = coeffs(1);
                    // a[2] = coeffs(2);
                    // a[3] = coeffs(3);

                    //생성한 qubic spline을 이용한 y 좌표 생성
                    // for(int j = 0 ; x.size() > j ; ++j){
                    //     double result = a[3] * pow(j,3) + a[2] * pow(j,2) + a[1] * j + a[0];
                    //     y.push_back(result);
                    //     // cout<<j<<" : "<<result<<endl;
                    // }


                    

                    for(int k = 0 ; y.size() > k ; ++k){
                        // cout<<x[k]<<","<<y[k]<<endl;
                        Matrix<double, 3, 1> local_result;
                        local_result << x[k],
                                        y[k],
                                           1;
                        
                        // cout<<"x:"<<local_result(0,0)<<"y:"<<local_result(1,0)<<endl;

                        Matrix<double, 3, 1> global_result; // 전역좌표계상에 만들어진 lattice 경로의 좌표들
                        global_result = trans_matrix * local_result;

                        geometry_msgs::PoseStamped read_pose;
                        read_pose.pose.position.x = global_result(0,0);
                        read_pose.pose.position.y = global_result(1,0);
                        read_pose.pose.position.z = 0;
                        read_pose.pose.orientation.x = 0;
                        read_pose.pose.orientation.y = 0;
                        read_pose.pose.orientation.z = 0;
                        read_pose.pose.orientation.w = 1;
                        // cout<<global_result<<endl; //문제부분
                        lattice_path.poses.push_back(read_pose);
                    }

                    lat_path.push_back(lattice_path);
                }

                // 끝점 넣어주기
                int add_point_size = min(6, static_cast<int>(local_path.poses.size()));
                // 기울기 시작점을 lattice 시작점으로 해야하나?
                for(int i = add_point_size + 1 ; look_distance * 2 > i ; ++i){
                    if(i+1 < local_path.poses.size()){
                        double tmp_theta = atan2(local_path.poses[i+1].pose.position.y - local_path.poses[i].pose.position.y,
                                                 local_path.poses[i+1].pose.position.x - local_path.poses[i].pose.position.x);
                        Matrix3d tmp_t;
                        tmp_t << cos(tmp_theta), -sin(tmp_theta), local_path.poses[i].pose.position.x, 
                                 sin(tmp_theta),  cos(tmp_theta), local_path.poses[i].pose.position.y, 
                                              0,               0,                                   1;

                        for(int lane_num = 0 ; lane_off_set.size() > lane_num ; ++lane_num){
                            Matrix<double, 3, 1> local_result;
                            local_result <<                      0,
                                            lane_off_set[lane_num],
                                                                 1;
                            Matrix<double, 3, 1> global_result;
                            global_result = tmp_t * local_result;

                            geometry_msgs::PoseStamped read_pose;

                            read_pose.pose.position.x = global_result(0,0);
                            read_pose.pose.position.y = global_result(1,0);
                            read_pose.pose.position.z = 0;
                            read_pose.pose.orientation.x = 0;
                            read_pose.pose.orientation.y = 0;
                            read_pose.pose.orientation.z = 0;
                            read_pose.pose.orientation.w = 1;
                            // cout<<global_result<<endl;
                            lat_path[lane_num].poses.push_back(read_pose);
                        }
                    }
                }


                // cout<<lat_path.size()<<endl;

                for(int l = 0 ; lat_path.size() > l ; ++l){
                    string topic_name = "/lattice_path_" + to_string(l+1);
                    ros::Publisher latpub = nh.advertise<nav_msgs::Path>(topic_name, 1);
                    publishers.push_back(latpub);
                    publishers[l].publish(lat_path[l]);
                }






            }

        }

        

        void readCsv(const string &file_name){
            path_folder_name = "path";
            string full_file_name = file_path + "/" + path_folder_name + "/" + file_name;
            ifstream openFile(full_file_name);

            global_path.header.frame_id = "/map";
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
                    global_path.poses.push_back(read_pose);

                    // cout<<read_pose<<endl;

                    


                    
                    
                }

                openFile.close();
                //publish global path 또는 global path에 저장
            }

            else{
                ROS_ERROR("Unable to open file");
            }

        }

        void findLocalPath(){
            nav_msgs::Path out_path;
            double current_x = status_msg.x;
            double current_y = status_msg.y;
            int current_waypoint = 0;

            double min_dis = std::numeric_limits<double>::infinity();

            path_pub.publish(global_path);

            // cout<<global_path.poses[60].pose.position.x<<endl;

            for(int i = 0 ; global_path.poses.size() > i ; ++i){
                double dx = current_x - global_path.poses[i].pose.position.x;
                double dy = current_y - global_path.poses[i].pose.position.y;
                double dis = sqrt(dx*dx + dy*dy);

                if(dis < min_dis){
                    min_dis = dis;
                    current_waypoint = i;
                    //가장 가까운 점을 current_waypoint로 선택
                }
            }

            prev_waypoint = current_waypoint;
            index = current_waypoint;

            int last_local_waypoint = 0; //local point가 생성될 때 끝점, 시작점은 현위치

            //끝부분에서 남은길이가 10보다 작을 때 처리하는 부분
            if(current_waypoint + 10 > global_path.poses.size()){
                last_local_waypoint = global_path.poses.size();
            }
            else{
                last_local_waypoint = current_waypoint + 10;
            }

            out_path.header.frame_id = "/map";

            for(int i = current_waypoint ; last_local_waypoint > i ; ++i){
                geometry_msgs::PoseStamped tmp_pose;
                tmp_pose.pose.position.x = global_path.poses[i].pose.position.x;
                // cout<<i<<": "<<global_path.poses[i].pose.position.x<<endl;
                tmp_pose.pose.position.y = global_path.poses[i].pose.position.y;
                tmp_pose.pose.position.z = global_path.poses[i].pose.position.z;
                tmp_pose.pose.orientation.x = 0;
                tmp_pose.pose.orientation.y = 0;
                tmp_pose.pose.orientation.z = 0;
                tmp_pose.pose.orientation.w = 0;
                tmp_pose.header.seq = index; // index저장 필요시 사용
                out_path.poses.push_back(tmp_pose);
            }
            if(abs(current_waypoint - last_local_waypoint) < 5){
                for(int i = 0 ; 5 > i ; ++i){
                    geometry_msgs::PoseStamped add_pose;
                    add_pose.pose.position.x = global_path.poses[i].pose.position.x;
                    add_pose.pose.position.y = global_path.poses[i].pose.position.y;
                    add_pose.pose.position.z = global_path.poses[i].pose.position.z;
                    add_pose.pose.orientation.x = 0;
                    add_pose.pose.orientation.y = 0;
                    add_pose.pose.orientation.z = 0;
                    add_pose.pose.orientation.w = 0;
                    add_pose.header.seq = index; // index저장 필요시 사용
                    out_path.poses.push_back(add_pose);
                }
            }

            cout<<out_path<<endl;
            local_path = out_path; // ->auto drive에서 사용하도록 publish & lattice planner에서 사용
            local_pub.publish(local_path);
        }

        void finalPathFinder(){
            findLocalPath();
            if(checkObject() == true){
                if(!isnan(status_msg.x)){
                    latticePlanner();
                    collisionCheck(lat_path);
                    lattice_pub.publish(lat_path[selected_lane]);
                }
                else{
                    
                }
            }

            else{
                lattice_pub.publish(local_path);
            }
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");

    pathreader pr;
    pr.readCsv("test.csv");

    ros::Rate loop_rate(50);
    // while(ros::ok()){
        
    //     pr.findLocalPath();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    while(ros::ok()){

        // pr.findLocalPath();
        // if(pr.checkObject() == true){
        //     if(!isnan(pr.status_msg.position.x)){
        //         pr.latticePlanner();
        //         pr.collisionCheck(pr.lat_path);
        //         pr.lattice_pub.publish(pr.lat_path[pr.selected_lane]);
        //     }
        //     else{
        //         continue;
        //     }
        // }

        // else{
        //     pr.lattice_pub.publish(pr.local_path);
        // }
        // pr.buildWall();
        pr.finalPathFinder();
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
