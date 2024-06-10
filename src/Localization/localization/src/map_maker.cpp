#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#define ORIGIN_LAT 37.544322 // 삼각지 x좌표
#define ORIGIN_LON 127.078958 // 삼각지 y좌표

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "csv_to_osm_node");
    ros::NodeHandle nh;
    lanelet::Origin origin({ORIGIN_LAT, ORIGIN_LON}); 

    // CSV 파일 경로
    std::string csv_file_path = "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/Monday4.csv";
    // OSM 파일 경로
    std::string osm_file_path = "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/Monday4.osm";
    // 수정된 CSV 파일 경로
    std::string modified_csv_file_path = "/home/eonsoo/2024_graduation_project/src/Localization/localization/map/Monday4_xy.csv";

    // OSM 파일 열기
    std::ofstream osm_file(osm_file_path);

    // 파일 열기 성공 여부 확인
    if (!osm_file.is_open()) {
        std::cout << "OSM 파일을 열 수 없습니다." << std::endl;
        return 1;
    }

    // 수정된 CSV 파일 열기 
    std::ofstream modified_csv_file(modified_csv_file_path);

    // 파일 열기 성공 여부 확인
    if (!modified_csv_file.is_open()) {
        std::cout << "수정된 CSV 파일을 열 수 없습니다." << std::endl;
        return 1;
    }

    // XML 선언 및 <osm> 태그 추가
    osm_file << "<?xml version='1.0' encoding='UTF-8'?>\n";
    osm_file << "<osm version='0.6' generator='JOSM'>\n";

    // CSV 파일 열기
    std::ifstream csv_file(csv_file_path);

    // 파일 열기 성공 여부 확인
    if (!csv_file.is_open()) {
        std::cout << "CSV 파일을 열 수 없습니다." << std::endl;
        return 1;
    }

    // 이전에 읽은 위도와 경도 값 저장 변수
    std::string prev_latitude_str, prev_longitude_str;
    double prev_x = 0.0, prev_y = 0.0;
    bool is_first_point = true; // 첫 번째 점 여부를 확인하는 변수
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
            std::cout << "이전 라인과 현재 라인의 값이 동일합니다. 건너뜁니다." << std::endl;
            continue;
        }

        // GPS 좌표를 UTM 좌표로 변환
        lanelet::GPSPoint gps_point;
        gps_point.lat = std::stod(latitude_str);
        gps_point.lon = std::stod(longitude_str);
        
        lanelet::projection::UtmProjector projection(origin);
        lanelet::BasicPoint3d point_3d = projection.forward(gps_point);
        lanelet::BasicPoint2d point(point_3d.x(), point_3d.y());

        double dist = sqrt(pow((point.x() - prev_x), 2) + pow((point.y() - prev_y), 2));
        if (is_first_point || dist >= 0.5) {
            // OSM 파일에 노드 정보 저장
            osm_file << "\t<node id='" << node_id << "' visible='true' version='1' lat='" << latitude_str << "' lon='" << longitude_str << "'/>\n";

            // 수정된 CSV 파일에 x, y 좌표 값 저장
            modified_csv_file << point.x() << "," << point.y() << std::endl;

            // 현재 값을 이전 값으로 업데이트
            prev_x = point.x();
            prev_y = point.y();
            is_first_point = false; // 첫 번째 점 저장 완료
            ++node_id; // 노드 ID 증가
        } else {
            std::cout << "이전 점과 현재 점의 거리가 0.5m 미만입니다. 건너뜁니다." << std::endl;
        }
    }

    // CSV 파일 닫기
    csv_file.close();

    // <osm> 태그 닫기
    osm_file << "</osm>\n";

    // OSM 파일 닫기
    osm_file.close();

    // 수정된 CSV 파일 닫기
    modified_csv_file.close();

    std::cout << "OSM 파일이 성공적으로 생성되었습니다: " << osm_file_path << std::endl;
    std::cout << "수정된 CSV 파일이 성공적으로 생성되었습니다: " << modified_csv_file_path << std::endl;

    // ROS 스핀
    ros::spin();

    return 0;
}
