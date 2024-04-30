#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "csv_to_osm_node");
    ros::NodeHandle nh;

    // CSV 파일 경로
    std::string csv_file_path = "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/map_v3.csv";
    // OSM 파일 경로
    std::string osm_file_path = "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/map_v3.osm";

    // OSM 파일 열기
    std::ofstream osm_file(osm_file_path);

    // 파일 열기 성공 여부 확인
    if (!osm_file.is_open()) {
        ROS_ERROR("OSM 파일을 열 수 없습니다.");
        return 1;
    }

    // XML 선언 및 <osm> 태그 추가
    osm_file << "<?xml version='1.0' encoding='UTF-8'?>\n";
    osm_file << "<osm version='0.6' generator='JOSM'>\n";

    // CSV 파일 열기
    std::ifstream csv_file(csv_file_path);

    // 파일 열기 성공 여부 확인
    if (!csv_file.is_open()) {
        ROS_ERROR("CSV 파일을 열 수 없습니다.");
        return 1;
    }

    // 이전에 읽은 위도와 경도 값 저장 변수
    std::string prev_latitude_str, prev_longitude_str;
    int node_id = 1;

    // CSV 파일을 한 줄씩 읽음
    std::string line;
    while (std::getline(csv_file, line)) {
        // 쉼표로 구분된 값을 읽기 위한 문자열 스트림 생성
        std::istringstream iss(line);
        std::string token;

        // 쉼표를 구분자로 사용하여 값을 읽음
        std::string latitude_str, longitude_str;

        // 쉼표를 기준으로 값을 분리하여 위도와 경도 값을 읽음
        std::getline(iss, latitude_str, ',');
        std::getline(iss, longitude_str, ',');

        // 이전에 읽은 값과 현재 값이 같으면 건너뜀
        if (latitude_str == prev_latitude_str && longitude_str == prev_longitude_str) {
            ROS_WARN("이전 라인과 현재 라인의 값이 동일합니다. 건너뜁니다.");
            continue;
        }

        // OSM 파일에 노드 정보 저장
        osm_file << "\t<node id='" << node_id << "' visible='true' version='1' lat='" << latitude_str << "' lon='" << longitude_str << "'/>\n";

        // 현재 값을 이전 값으로 업데이트
        prev_latitude_str = latitude_str;
        prev_longitude_str = longitude_str;

        // 노드 ID 증가
        ++node_id;
    }

    // CSV 파일 닫기
    csv_file.close();

    // <osm> 태그 닫기
    osm_file << "</osm>\n";

    // OSM 파일 닫기
    osm_file.close();

    ROS_INFO("OSM 파일이 성공적으로 생성되었습니다: %s", osm_file_path.c_str());

    // ROS 스핀
    ros::spin();

    return 0;
}