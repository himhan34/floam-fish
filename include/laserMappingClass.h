// FLOAM의 저자: Wang Han
// 이메일: wh200720041@gmail.com
// 홈페이지: https://wanghan.pro

#ifndef _LASER_MAPPING_H_
#define _LASER_MAPPING_H_

// PCL (Point Cloud Library) 헤더 파일 포함
#include <pcl/point_cloud.h> // 포인트 클라우드 데이터 구조
#include <pcl/point_types.h> // 다양한 포인트 타입 정의
#include <pcl/filters/filter.h> // 필터링 관련 함수
#include <pcl/filters/voxel_grid.h> // Voxel 그리드 필터 (다운샘플링)
#include <pcl/filters/passthrough.h> // PassThrough 필터 (특정 축 기준으로 필터링)
#include <pcl_ros/impl/transforms.hpp> // PCL과 ROS 간의 변환 함수 구현

// Eigen 라이브러리 헤더 파일 포함
#include <Eigen/Dense> // 선형 대수 연산을 위한 헤더
#include <Eigen/Geometry> // 기하학적 변환을 위한 헤더

// C++ 표준 라이브러리 헤더 파일 포함
#include <string> // 문자열 처리
#include <math.h> // 수학 함수 (예: 삼각함수, 제곱근 등)
#include <vector> // 벡터 자료구조

// 레이저 셀의 크기 정의
#define LASER_CELL_WIDTH 50.0 // 레이저 셀 너비 (단위: m)
#define LASER_CELL_HEIGHT 50.0 // 레이저 셀 높이 (단위: m)
#define LASER_CELL_DEPTH 50.0 // 레이저 셀 깊이 (단위: m)

// 맵을 여러 개의 서브 포인트 클라우드로 나누기 위한 범위 정의
#define LASER_CELL_RANGE_HORIZONTAL 2 // 수평 방향 셀 범위
#define LASER_CELL_RANGE_VERTICAL 2 // 수직 방향 셀 범위

// LaserMappingClass 클래스 정의
class LaserMappingClass 
{
    public:
        // 생성자: 클래스의 초기 설정을 담당합니다.
        LaserMappingClass();
        
        // 초기화 함수: 맵의 해상도를 설정합니다.
        void init(double map_resolution);

        // 현재 포인트를 맵에 업데이트하는 함수
        void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current);

        // 맵을 반환하는 함수
        pcl::PointCloud<pcl::PointXYZI>::Ptr getMap(void);

    private:
        int origin_in_map_x; // 맵 내의 원점 x 좌표
        int origin_in_map_y; // 맵 내의 원점 y 좌표
        int origin_in_map_z; // 맵 내의 원점 z 좌표
        int map_width; // 맵의 너비
        int map_height; // 맵의 높이
        int map_depth; // 맵의 깊이

        // 3D 벡터로 구성된 맵, 각 셀에 포인트 클라우드가 저장됩니다.
        std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map;

        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 다운샘플링 필터 (Voxel 그리드 사용)

        // 맵의 너비를 음수 방향으로 확장하는 함수
        void addWidthCellNegative(void);
        
        // 맵의 너비를 양수 방향으로 확장하는 함수
        void addWidthCellPositive(void);

        // 맵의 높이를 음수 방향으로 확장하는 함수
        void addHeightCellNegative(void);

        // 맵의 높이를 양수 방향으로 확장하는 함수
        void addHeightCellPositive(void);

        // 맵의 깊이를 음수 방향으로 확장하는 함수
        void addDepthCellNegative(void);

        // 맵의 깊이를 양수 방향으로 확장하는 함수
        void addDepthCellPositive(void);

        // 포인트 클라우드가 맵 범위 내에 있는지 확인하는 함수
        void checkPoints(int& x, int& y, int& z);
};

#endif // _LASER_MAPPING_H_
