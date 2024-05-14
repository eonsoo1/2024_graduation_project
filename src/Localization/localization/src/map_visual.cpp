#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <string>

// CSV 파일을 파싱하고 마커 배열로 발행하는 함수
void parseCSVAndPublishMarkers(const std::string& filename, ros::Publisher& marker_pub) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Could not open CSV file: %s", filename.c_str());
        return;
    }

    visualization_msgs::MarkerArray marker_array;
    std::string line;
    int id = 0;

    // CSV 파일의 각 라인을 읽음
    while (std::getline(file, line)) {
        std::stringstream line_stream(line);
        std::string cell;
        double x, y;

        // x 값을 읽음
        if (std::getline(line_stream, cell, ',')) {
            x = std::stod(cell);
        } else {
            continue;
        }

        // y 값을 읽음
        if (std::getline(line_stream, cell, ',')) {
            y = std::stod(cell);
        } else {
            continue;
        }

        // 이 점에 대한 마커 생성
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";  // 마커의 좌표계 프레임 설정
        marker.header.stamp = ros::Time::now();
        marker.ns = "csv_points";  // 네임스페이스 설정
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;  // 마커 타입 설정
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;  // 마커 크기 설정
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;  // 마커 색상 설정 (알파값 포함)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);  // 마커 배열에 추가
    }

    file.close();

    marker_pub.publish(marker_array);  // 마커 배열을 발행
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_to_markers");  // ROS 노드 초기화
    ros::NodeHandle nh;

    std::string filename;
    nh.param<std::string>("csv_filename", filename, "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/map_v_test_xy.csv");  // CSV 파일 이름 파라미터 읽기

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);  // 마커 배열 발행자 설정

    ros::Rate loop_rate(1);  // 1Hz로 루프 설정

    while (ros::ok()) {
        parseCSVAndPublishMarkers(filename, marker_pub);  // CSV 파일 파싱 및 마커 발행
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
