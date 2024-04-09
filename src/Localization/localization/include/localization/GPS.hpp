#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_broadcaster.h>

#include <string>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <thread>


#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#define ORIGIN_LAT 37.542608//330093 // 삼각지 x좌표
#define ORIGIN_LON 127.076774//4156806 // 삼각지 y좌표

using namespace std;
 
class GPS{
    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        ros::Publisher utm_pub;
        ros::Subscriber gps_sub;

        visualization_msgs::Marker gps_path;
        tf::TransformBroadcaster tfcaster;

        lanelet::Origin m_origin; 
        lanelet::BasicPoint2d m_utm_point;
        lanelet::GPSPoint m_gps_point;

        geometry_msgs::Point m_origin_coord;
        geometry_msgs::Point m_utm_coord;
        float m_lat;
        float m_lon;
        float m_alt;
        double m_utm_x;
        double m_utm_y;
        double m_utm_z;

    public :
        GPS();
        ~GPS(){};
        void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data_msg);
        void Print();
        
};
