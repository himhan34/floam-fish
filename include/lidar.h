// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_  // _LIDAR_H_가 정의되지 않은 경우에만 이 코드를 실행합니다.
#define _LIDAR_H_  // _LIDAR_H_를 정의하여 중복 포함을 방지합니다.

// LiDAR 파라미터 정의

namespace lidar {  // lidar라는 네임스페이스를 정의합니다.

class Lidar {
    public:
        Lidar();  // 기본 생성자 선언

        void setScanPeriod(double scan_period_in);  // 스캔 주기를 설정하는 함수
        void setLines(double num_lines_in);  // 라인의 개수를 설정하는 함수
        void setVerticalAngle(double vertical_angle_in);  // 수직 각도를 설정하는 함수
        void setVerticalResolution(double vertical_angle_resolution_in);  // 수직 해상도를 설정하는 함수
        void setMaxDistance(double max_distance_in);  // 최대 거리를 설정하는 함수 (기본값은 100으로 변경하지 않음)
        void setMinDistance(double min_distance_in);  // 최소 거리를 설정하는 함수

        double max_distance;  // 최대 거리 (기본값: 100)
        double min_distance;  // 최소 거리
        int num_lines;  // 라인의 개수
        double scan_period;  // 스캔 주기
        int points_per_line;  // 라인당 점의 개수
        double horizontal_angle_resolution;  // 수평 각도 해상도
        double horizontal_angle;  // 수평 각도
        double vertical_angle_resolution;  // 수직 각도 해상도
        double vertical_angle;  // 수직 각도
};

}  // 네임스페이스 lidar 끝

#endif // _LIDAR_H_  // _LIDAR_H_ 정의 끝
