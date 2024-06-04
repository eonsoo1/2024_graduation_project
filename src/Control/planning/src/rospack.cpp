#include <ros/package.h>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>

#include <typeinfo>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class pathreader{
    protected:
        string file_path;
        string path_folder_name;
        nav_msgs::Path out_path;

        ros::NodeHandle nh;
        ros::Publisher path_pub;

        
    
    public:
        pathreader(){
            file_path = ros::package::getPath("planning");

            path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);

        }
        ~pathreader(){}

        void pub(const nav_msgs::Path &path){
            path_pub.publish(path);
        }

        

        nav_msgs::Path read_csv(const string &file_name){
            path_folder_name = "path";
            string full_file_name = file_path + "/" + path_folder_name + "/" + file_name;
            ifstream openFile(full_file_name);

            out_path.header.frame_id = "/map";
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
                    read_pose.pose.position.x = stof(row[1]);
                    read_pose.pose.position.y = stof(row[2]);
                    read_pose.pose.position.z = 0;
                    read_pose.pose.orientation.x = 0;
                    read_pose.pose.orientation.y = 0;
                    read_pose.pose.orientation.z = 0;
                    read_pose.pose.orientation.w = 0;
                    out_path.poses.push_back(read_pose);
                    cout<<read_pose<<endl;
                    
                    
                }

                openFile.close();

                return out_path;
            }

            else{
                ROS_ERROR("Unable to open file");
            }

        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "pathread");
    pathreader pr;
    nav_msgs::Path global_path;
    string filename = "map_v2.csv";
    global_path = pr.read_csv(filename);


    ros::Rate loop_rate(50);
    while(ros::ok()){
        pr.pub(global_path);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}