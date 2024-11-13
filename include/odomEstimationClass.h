// FLOAM의 작성자: Wang Han
// 이메일: wh200720041@gmail.com
// 홈페이지: https://wanghan.pro

#ifndef _ODOM_ESTIMATION_CLASS_H_  // _ODOM_ESTIMATION_CLASS_H_가 정의되지 않은 경우에만 이 코드를 실행합니다.
#define _ODOM_ESTIMATION_CLASS_H_  // _ODOM_ESTIMATION_CLASS_H_를 정의하여 중복 포함을 방지합니다.

// 표준 라이브러리
#include <string>  // 문자열 처리에 필요한 헤더 파일 포함
#include <math.h>  // 수학 연산에 필요한 헤더 파일 포함
#include <vector>  // 벡터 컨테이너를 사용하기 위한 헤더 파일 포함

// PCL (Point Cloud Library)
#include <pcl/point_cloud.h>  // 포인트 클라우드 관련 헤더 파일 포함
#include <pcl/point_types.h>  // 포인트 타입 관련 헤더 파일 포함
#include <pcl/filters/filter.h>  // 필터링 함수 헤더 파일 포함
#include <pcl/filters/voxel_grid.h>  // Voxel Grid 필터 헤더 파일 포함
#include <pcl/filters/passthrough.h>  // Passthrough 필터 헤더 파일 포함
#include <pcl/kdtree/kdtree_flann.h>  // Kd-tree 구조 헤더 파일 포함
#include <pcl/filters/statistical_outlier_removal.h>  // 통계적 외부점 제거 필터 헤더 포함
#include <pcl/filters/extract_indices.h>  // 인덱스 추출 필터 헤더 포함
#include <pcl/filters/crop_box.h>  // 크롭 박스 필터 헤더 파일 포함

// Ceres Solver
#include <ceres/ceres.h>  // Ceres 최적화 라이브러리 헤더 파일 포함
#include <ceres/rotation.h>  // Ceres 회전 관련 함수 포함

// Eigen 라이브러리
#include <Eigen/Dense>  // Eigen 밀집 행렬 관련 헤더 파일 포함
#include <Eigen/Geometry>  // Eigen 기하학적 변환 관련 헤더 파일 포함

// 로컬 라이브러리
#include "lidar.h"  // LiDAR 관련 헤더 파일 포함
#include "lidarOptimization.h"  // LiDAR 최적화 관련 헤더 파일 포함
#include <ros/ros.h>  // ROS 헤더 파일 포함

// OdomEstimationClass 클래스 정의
class OdomEstimationClass 
{
    public:
        OdomEstimationClass();  // 기본 생성자 선언

        // LiDAR 파라미터 및 맵 해상도를 초기화하는 함수
        void init(lidar::Lidar lidar_param, double map_resolution);
        // 포인트로 맵을 초기화하는 함수
        void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
        // 맵에 포인트를 업데이트하는 함수
        void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
        // 현재 맵을 가져오는 함수
        void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap);

        Eigen::Isometry3d odom;  // 변환 행렬 (Isometry3d)로 나타낸 오도메트리
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;  // 코너 포인트 맵
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;  // 서프 포인트 맵

    private:
        // 최적화 변수
        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};  // 초기화된 매개 변수 배열
        Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);  // 쿼터니언 맵핑
        Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);  // 벡터 맵핑

        Eigen::Isometry3d last_odom;  // 마지막 오도메트리

        // Kd-tree
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;  // 엣지 맵에 대한 Kd-tree
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;  // 서프 맵에 대한 Kd-tree

        // 맵에 추가하기 전에 포인트를 다운샘플링
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;  // 엣지 포인트 다운샘플러
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;  // 서프 포인트 다운샘플러

        // 로컬 맵
        pcl::CropBox<pcl::PointXYZI> cropBoxFilter;  // 크롭 박스 필터

        // 최적화 횟수
        int optimization_count;  // 최적화 실행 횟수

        // 함수
        void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);  // 엣지 코스트 팩터 추가
        void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);  // 서프 코스트 팩터 추가
        void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);  // 맵에 포인트 추가
        void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);  // 포인트를 맵에 연관시키는 함수
        void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);  // 다운샘플링하여 맵에 추가
};

#endif // _ODOM_ESTIMATION_CLASS_H_  // _ODOM_ESTIMATION_CLASS_H_ 정의 끝
