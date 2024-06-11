#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>
#include <numeric>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <limits>
#include <array>


#include <autonomous_msgs/ObjectStatusList.h>
#include <autonomous_msgs/PoseMsg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


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
        ros::Subscriber vehicle_status_sub;
        ros::Subscriber object_status_sub;

        //subscribe msg
        autonomous_msgs::PoseMsg status_msg;
        autonomous_msgs::ObjectStatusList object_data;


        //variable
        string file_path;
        string path_folder_name;
        nav_msgs::Path global_path;
        nav_msgs::Path local_path;
        nav_msgs::Path lattice_path;
        // autonomous_msgs::LatticePathData lattice_lane;


        int prev_waypoint = 0;
        int index = 0;

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

            vehicle_status_sub = nh.subscribe
                                ("/m_vehicle_pose", 10, &pathreader::status_callback, this);

            object_status_sub = nh.subscribe
                                ("/object_center", 10, &pathreader::object_callback, this);

        }
        ~pathreader(){}

        void pub(const nav_msgs::Path &path){
            path_pub.publish(path);
        }

        void status_callback(const autonomous_msgs::PoseMsg::ConstPtr &msg){
            status_msg = *msg;
        }

        void object_callback(const autonomous_msgs::ObjectStatusList::ConstPtr &msg){
            object_data = *msg;
        }


        // void checkObject(){
        //     is_crash = false;
        //     double dis = 0.0;

        //     for(int obstacle = 0 ; object_data.obstacle_list.size() > obstacle ; ++obstacle){
        //         for(int path = 0 ; local_path.poses.size() > path ; ++path){
        //             dis = sqrt(pow(local_path.poses[path].pose.position.x 
        //                        - object_data.obstacle_list[obstacle].position.x, 2)
        //                      + pow(local_path.poses[path].pose.position.y 
        //                        - object_data.obstacle_list[obstacle].position.y, 2));

        //             cout<<dis<<endl;

        //             if(dis < 3){
        //                 is_crash = true;
        //                 break;
        //             }
        //         }
        //     }
        // }


        // void collisionCheck(){

        //     selected_lane = -1;

        //     array<double, 8> lane_weight = {4, 3, 2, 1, 1, 2, 3, 4};
        //     double dis = 0.0;

        //     for(int obstacle = 0 ; object_data.obstacle_list.size() > obstacle ; ++obstacle){
        //         for(int lane = 0 ; lattice_lane.size() > lane ; ++lane){
        //             for(int pos = 0 ; lattice_lane.latticeLanes[lane].poses.size() > pos ; ++pos){
        //                 dis = sqrt(pow(object_data.obstacle_list[obstacle].position.x
        //                                 - lattice_lane.latticeLanes[lane].poses[pos].pose.position.x,2)
        //                          + pow(object_data.obstacle_list[obstacle].position.y
        //                                 - lattice_lane.latticeLanes[lane].poses[pos].pose.position.y,2));
                        
        //                 if(dis < 3){
        //                     lane_weight[lane] = lane_weight[lane] + 100;
        //                 }
        //             }
        //         }
        //     }

        //     lane_weight[0] = (lane_weight[0]+lane_weight[1]) / 2;
        //     lane_weight[1] = (lane_weight[1]+lane_weight[2]) / 2;
        //     lane_weight[2] = (lane_weight[2]+lane_weight[3]) / 2;
        //     lane_weight[7] = (lane_weight[6]+lane_weight[7]) / 2;
        //     lane_weight[5] = (lane_weight[4]+lane_weight[5]) / 2;
        //     lane_weight[6] = (lane_weight[5]+lane_weight[6]) / 2;

        //     auto minIndex = min_element(lane_weight.begin(), lane_weight.end());
        //     selected_lane = distance(lane_weight.begin(), minIndex); //lane index 반환됨

        // }

        // void latticePlanner(){
        //     double pose_x = status_msg.x + 1.26;
        //     double pose_y = status_msg.y;
        //     double velocity = status_msg.velocity * 3.6;

        //     int look_distance = static_cast<int>(velocity * 0.2 * 2);

        //     if(look_distance < 8){
        //         look_distance = 8;
        //     }

        //     if(local_path.poses.size() > look_distance){
                
        //     }

        // }

        

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

                    cout<<read_pose<<endl;

                    


                    
                    
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

            for(int i = 0 ; global_path.poses.size() > i ; ++i){
                double dx = current_x - global_path.poses[i].pose.position.x;
                double dy = current_y - global_path.poses[i].pose.position.y;
                double dis = sqrt(dx*dx + dy*dy);

                if(dis < min_dis){
                    min_dis = dis;
                    current_waypoint = i;
                }
            }

            prev_waypoint = current_waypoint;
            index = current_waypoint;

            int last_local_waypoint = 0; //local point가 생성될 때 끝점, 시작점은 현위치

            if(current_waypoint + 10 > global_path.poses.size()){
                last_local_waypoint = global_path.poses.size();
            }
            else{
                last_local_waypoint = current_waypoint + 10;
            }

            out_path.header.frame_id = "world";

            for(int i = current_waypoint ; last_local_waypoint > i ; ++i){
                geometry_msgs::PoseStamped tmp_pose;
                tmp_pose.pose.position.x = global_path.poses[i].pose.position.x;
                tmp_pose.pose.position.y = global_path.poses[i].pose.position.y;
                tmp_pose.pose.position.z = global_path.poses[i].pose.position.z;
                tmp_pose.pose.orientation.x = 0;
                tmp_pose.pose.orientation.y = 0;
                tmp_pose.pose.orientation.z = 0;
                tmp_pose.pose.orientation.w = 0;
                tmp_pose.header.seq = index; // index저장 필요시 사용
                out_path.poses.push_back(tmp_pose);
            }


            local_path = out_path; // ->auto drive에서 사용하도록 publish & lattice planner에서 사용
            local_pub.publish(local_path);
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");

    pathreader pr;
    pr.readCsv("Nocheon_xy.csv");

    ros::Rate loop_rate(50);
    while(ros::ok()){
        
        pr.findLocalPath();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}