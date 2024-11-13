// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// C++ 표준 라이브러리 헤더 파일 포함
#include <cmath> // 수학 함수 (예: 삼각함수, 제곱근 등)
#include <vector> // 벡터 자료구조
#include <mutex> // 뮤텍스(스레드 동기화) 사용
#include <queue> // 큐 자료구조
#include <thread> // 멀티스레딩 지원
#include <chrono> // 시간 관련 기능

// ROS 라이브러리 헤더 파일 포함
#include <ros/ros.h> // ROS 기본 기능
#include <sensor_msgs/Imu.h> // IMU 센서 메시지 데이터 타입
#include <sensor_msgs/PointCloud2.h> // 센서 메시지 포인트 클라우드 데이터 타입
#include <nav_msgs/Odometry.h> // 네비게이션 메시지 오도메트리 데이터 타입
#include <tf/transform_datatypes.h> // 좌표 변환 데이터 타입
#include <tf/transform_broadcaster.h> // 좌표 변환 브로드캐스터

// PCL (Point Cloud Library) 헤더 파일 포함
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 간 메시지 변환
#include <pcl/point_cloud.h> // 포인트 클라우드 데이터 구조
#include <pcl/point_types.h> // 다양한 포인트 타입 정의

// 로컬 헤더 파일 포함
#include "lidar.h" // 라이다 관련 클래스 및 함수 정의
#include "laserProcessingClass.h" // 레이저 데이터 처리 클래스 정의

// 레이저 데이터 처리 클래스의 인스턴스 생성
LaserProcessingClass laserProcessing;

// 스레드 동기화를 위한 뮤텍스 객체 생성
std::mutex mutex_lock;

// 포인트 클라우드 데이터를 저장하는 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

// Lidar 클래스의 인스턴스를 생성하여 Lidar 파라미터를 저장
lidar::Lidar lidar_param;

// 퍼블리셔 객체 선언: 에지 포인트, 서프 포인트, 필터링된 포인트 클라우드를 퍼블리시
ros::Publisher pubEdgePoints; // 에지 포인트 퍼블리셔
ros::Publisher pubSurfPoints; // 서프 포인트 퍼블리셔
ros::Publisher pubLaserCloudFiltered; // 필터링된 포인트 클라우드 퍼블리셔

// 벨로다인 포인트 클라우드를 처리하는 콜백 함수
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock(); // 뮤텍스를 사용하여 큐에 안전하게 접근
    pointCloudBuf.push(laserCloudMsg); // 포인트 클라우드를 버퍼에 추가
    mutex_lock.unlock(); // 뮤텍스 잠금 해제
}

// 오도메트리 추정 시간의 총합
double total_time = 0;
// 처리한 총 프레임 수
int total_frame = 0;

void laser_processing(){
    while(1){
        // 포인트 클라우드 버퍼가 비어 있지 않은 경우
        if(!pointCloudBuf.empty()){
            // 데이터 읽기
            mutex_lock.lock(); // 뮤텍스 잠금
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in); // ROS 메시지를 PCL 포인트 클라우드로 변환
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp; // 포인트 클라우드의 타임스탬프 저장
            pointCloudBuf.pop(); // 버퍼에서 포인트 클라우드를 제거
            mutex_lock.unlock(); // 뮤텍스 잠금 해제

            // 에지 및 서프 포인트 클라우드를 저장할 포인터 생성
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            // 레이저 처리 시간을 측정합니다.
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now(); // 시작 시간 기록
            laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf); // 특징 추출
            end = std::chrono::system_clock::now(); // 종료 시간 기록
            std::chrono::duration<float> elapsed_seconds = end - start; // 경과 시간 계산
            total_frame++; // 처리한 프레임 수 증가
            float time_temp = elapsed_seconds.count() * 1000; // 경과 시간을 밀리초로 변환
            total_time += time_temp; // 총 시간 누적
            // ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame); // 평균 처리 시간 출력 (주석 처리됨)

            // 필터링된 포인트 클라우드 메시지를 생성하고 퍼블리시합니다.
            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered += *pointcloud_edge; // 에지 포인트를 필터링된 포인트 클라우드에 추가
            *pointcloud_filtered += *pointcloud_surf; // 서프 포인트를 필터링된 포인트 클라우드에 추가
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg); // PCL 포인트 클라우드를 ROS 메시지로 변환
            laserCloudFilteredMsg.header.stamp = pointcloud_time; // 타임스탬프 설정
            laserCloudFilteredMsg.header.frame_id = "base_link"; // 프레임 ID 설정
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg); // 필터링된 포인트 클라우드 퍼블리시

            // 에지 포인트 클라우드 메시지를 생성하고 퍼블리시합니다.
            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg); // PCL 포인트 클라우드를 ROS 메시지로 변환
            edgePointsMsg.header.stamp = pointcloud_time; // 타임스탬프 설정
            edgePointsMsg.header.frame_id = "base_link"; // 프레임 ID 설정
            pubEdgePoints.publish(edgePointsMsg); // 에지 포인트 퍼블리시

            // 서프 포인트 클라우드 메시지를 생성하고 퍼블리시합니다.
            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg); // PCL 포인트 클라우드를 ROS 메시지로 변환
            surfPointsMsg.header.stamp = pointcloud_time; // 타임스탬프 설정
            surfPointsMsg.header.frame_id = "base_link"; // 프레임 ID 설정
            pubSurfPoints.publish(surfPointsMsg); // 서프 포인트 퍼블리시
        }
        // 매 루프마다 2ms 동안 대기
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    // ROS 초기화: 노드 이름을 "main"으로 설정
    ros::init(argc, argv, "main");
    ros::NodeHandle nh; // ROS 노드 핸들 생성

    // Lidar 파라미터 초기값 설정
    int scan_line = 64; // 라이다 스캔 라인 수
    double vertical_angle = 2.0; // 라이다의 수직 각도
    double scan_period = 0.1; // 라이다 스캔 주기
    double max_dis = 60.0; // 라이다 최대 거리
    double min_dis = 2.0; // 라이다 최소 거리

    // ROS 파라미터 서버에서 설정값을 가져옵니다.
    nh.getParam("/scan_period", scan_period); // 스캔 주기 파라미터 가져오기
    nh.getParam("/vertical_angle", vertical_angle); // 수직 각도 파라미터 가져오기
    nh.getParam("/max_dis", max_dis); // 최대 거리 파라미터 가져오기
    nh.getParam("/min_dis", min_dis); // 최소 거리 파라미터 가져오기
    nh.getParam("/scan_line", scan_line); // 스캔 라인 수 파라미터 가져오기

    // Lidar 파라미터 설정
    lidar_param.setScanPeriod(scan_period); // 스캔 주기 설정
    lidar_param.setVerticalAngle(vertical_angle); // 수직 각도 설정
    lidar_param.setLines(scan_line); // 스캔 라인 수 설정
    lidar_param.setMaxDistance(max_dis); // 최대 거리 설정
    lidar_param.setMinDistance(min_dis); // 최소 거리 설정

    // 레이저 데이터 처리 클래스 초기화
    laserProcessing.init(lidar_param); // Lidar 파라미터로 초기화

    // 포인트 클라우드 데이터를 구독하는 ROS 서브스크라이버 생성
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler); // "/velodyne_points" 토픽 구독

    // 필터링된 포인트 클라우드를 퍼블리시하는 퍼블리셔 생성
    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100); // "/velodyne_points_filtered" 토픽 퍼블리시

    // 에지 포인트 클라우드를 퍼블리시하는 퍼블리셔 생성
    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100); // "/laser_cloud_edge" 토픽 퍼블리시

    // 서프 포인트 클라우드를 퍼블리시하는 퍼블리셔 생성
    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); // "/laser_cloud_surf" 토픽 퍼블리시

    // 레이저 처리 프로세스를 실행하는 스레드를 생성합니다.
    std::thread laser_processing_process{laser_processing}; // 레이저 처리 스레드 실행

    // ROS 이벤트 루프를 실행하여 콜백을 처리합니다.
    ros::spin(); // ROS 이벤트 루프 실행

    return 0; // 프로그램 종료
}
