// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

// PCL (Point Cloud Library) 헤더 파일 포함
#include <pcl/point_cloud.h> // 포인트 클라우드 데이터 구조
#include <pcl/point_types.h> // 포인트 타입 정의

// PCL 필터 관련 헤더 파일 포함
#include <pcl/filters/filter.h> // 필터링 관련 기본 함수
#include <pcl/filters/voxel_grid.h> // Voxel 그리드 필터 (다운샘플링)
#include <pcl/filters/passthrough.h> // PassThrough 필터 (특정 축 기준 필터링)
#include <pcl/filters/extract_indices.h> // 인덱스 기반 필터링
#include <pcl/filters/crop_box.h> // 박스 형태로 포인트 클라우드를 자르는 필터

// Lidar 관련 헤더 파일 포함
#include "lidar.h" // 라이다 클래스 및 관련 함수 정의

// 포인트의 공분산 정보를 저장하는 클래스
class Double2d {
public:
	int id; // 포인트의 ID
	double value; // 공분산 값
	Double2d(int id_in, double value_in); // 생성자: ID와 값을 설정
};

// 포인트 정보 클래스를 정의합니다.
class PointsInfo {
public:
	int layer; // 포인트가 속한 레이어
	double time; // 포인트의 시간 정보
	PointsInfo(int layer_in, double time_in); // 생성자: 레이어와 시간을 설정
};

// LaserProcessingClass 클래스 정의
class LaserProcessingClass {
public:
    LaserProcessingClass(); // 생성자: 클래스의 초기 설정을 담당합니다.
    
    // 라이다 파라미터를 초기화하는 함수
    void init(lidar::Lidar lidar_param_in);

    // 특징 추출 함수: 입력 포인트 클라우드에서 에지와 서프 특징을 추출
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, 
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, 
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

    // 섹터에서 특징을 추출하는 함수
    void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, 
                                     std::vector<Double2d>& cloudCurvature, 
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, 
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

private:
    lidar::Lidar lidar_param; // 라이다 파라미터를 저장하는 멤버 변수
};

#endif // _LASER_PROCESSING_CLASS_H_
