// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// "lidar.h" 헤더 파일 포함
#include "lidar.h"

// lidar 네임스페이스의 Lidar 클래스 생성자 정의
lidar::Lidar::Lidar() {
    // 생성자 초기화 (추가 설정 없음)
}

// 스캔 라인의 수를 설정하는 함수
void lidar::Lidar::setLines(double num_lines_in) {
    num_lines = num_lines_in;
}

// 라이다의 수직 각도를 설정하는 함수
void lidar::Lidar::setVerticalAngle(double vertical_angle_in) {
    vertical_angle = vertical_angle_in;
}

// 수직 해상도를 설정하는 함수
void lidar::Lidar::setVerticalResolution(double vertical_angle_resolution_in) {
    vertical_angle_resolution = vertical_angle_resolution_in;
}

// 스캔 주기를 설정하는 함수
void lidar::Lidar::setScanPeriod(double scan_period_in) {
    scan_period = scan_period_in;
}

// 최대 탐지 거리를 설정하는 함수
void lidar::Lidar::setMaxDistance(double max_distance_in) {
    max_distance = max_distance_in;
}

// 최소 탐지 거리를 설정하는 함수
void lidar::Lidar::setMinDistance(double min_distance_in) {
    min_distance = min_distance_in;
}
