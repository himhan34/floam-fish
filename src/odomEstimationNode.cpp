// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// C++ 표준 라이브러리 헤더 파일 포함
#include <cmath> // 수학 관련 함수 (ex. 삼각함수, 제곱근 등)
#include <vector> // 벡터 자료구조
#include <mutex> // 뮤텍스(동기화 도구) 사용
#include <queue> // 큐 자료구조
#include <thread> // 멀티스레딩 지원
#include <chrono> // 시간 관련 함수

// ROS 라이브러리 헤더 파일 포함
#include <ros/ros.h> // ROS 기본 기능
#include <sensor_msgs/PointCloud2.h> // 센서 메시지 포인트 클라우드 데이터 타입
#include <nav_msgs/Odometry.h> // 네비게이션 메시지 오도메트리 데이터 타입
#include <tf/transform_datatypes.h> // 좌표 변환 데이터 타입
#include <tf/transform_broadcaster.h> // 좌표 변환 브로드캐스터

// PCL (Point Cloud Library) 헤더 파일 포함
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 간 메시지 변환
#include <pcl/point_cloud.h> // 포인트 클라우드 데이터 구조
#include <pcl/point_types.h> // 다양한 포인트 타입 정의

// 로컬 헤더 파일 포함
#include "lidar.h" // Lidar 관련 클래스 및 함수
#include "odomEstimationClass.h" // 오도메트리 추정 클래스 정의

// 오도메트리 추정 클래스의 인스턴스 생성
OdomEstimationClass odomEstimation;

// 스레드 동기화를 위한 뮤텍스 객체 생성
std::mutex mutex_lock;

// 에지 포인트 클라우드를 저장하는 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;

// 서프 포인트 클라우드를 저장하는 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;

// Lidar 클래스의 인스턴스를 생성하여 Lidar 파라미터를 저장
lidar::Lidar lidar_param;

// 레이저 오도메트리를 퍼블리시할 ROS 퍼블리셔 객체
ros::Publisher pubLaserOdometry;

// 서프 포인트 클라우드 메시지를 처리하는 콜백 함수
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock(); // 뮤텍스를 사용하여 동기화
    pointCloudSurfBuf.push(laserCloudMsg); // 서프 포인트 클라우드를 버퍼에 추가
    mutex_lock.unlock(); // 뮤텍스 잠금 해제
}

// 에지 포인트 클라우드 메시지를 처리하는 콜백 함수
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock(); // 뮤텍스를 사용하여 동기화
    pointCloudEdgeBuf.push(laserCloudMsg); // 에지 포인트 클라우드를 버퍼에 추가
    mutex_lock.unlock(); // 뮤텍스 잠금 해제
}

// 오도메트리 초기화 상태 플래그
bool is_odom_inited = false;
// 오도메트리 추정 시간의 총합
double total_time = 0;
// 처리한 총 프레임 수
int total_frame = 0;

// 오도메트리 추정 함수
void odom_estimation(){
    while(1){
        // 에지와 서프 포인트 클라우드 버퍼가 비어 있지 않은 경우
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            // 데이터 읽기
            mutex_lock.lock(); // 뮤텍스 잠금
            // 서프 포인트 클라우드의 타임스탬프가 에지 포인트 클라우드보다 오래된 경우
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)){
                pointCloudSurfBuf.pop(); // 서프 포인트 클라우드를 버퍼에서 제거
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction"); // 경고 메시지 출력
                mutex_lock.unlock(); // 뮤텍스 잠금 해제
                continue; // 다음 루프로 이동
            }

            // 에지 포인트 클라우드의 타임스탬프가 서프 포인트 클라우드보다 오래된 경우
            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)){
                pointCloudEdgeBuf.pop(); // 에지 포인트 클라우드를 버퍼에서 제거
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction"); // 경고 메시지 출력
                mutex_lock.unlock(); // 뮤텍스 잠금 해제
                continue; // 다음 루프로 이동
            }

            // 타임스탬프가 정렬된 경우
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in); // ROS 메시지를 PCL 포인트 클라우드로 변환
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in); // ROS 메시지를 PCL 포인트 클라우드로 변환
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp; // 서프 포인트 클라우드의 타임스탬프 저장
            pointCloudEdgeBuf.pop(); // 에지 포인트 클라우드를 버퍼에서 제거
            pointCloudSurfBuf.pop(); // 서프 포인트 클라우드를 버퍼에서 제거
            mutex_lock.unlock(); // 뮤텍스 잠금 해제

            // 오도메트리 초기화가 안 되어 있는 경우
            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in); // 맵을 포인트로 초기화
                is_odom_inited = true; // 초기화 완료 플래그 설정
                ROS_INFO("odom inited"); // 정보 메시지 출력
            } else {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now(); // 시작 시간 기록
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in); // 맵에 포인트 업데이트
                end = std::chrono::system_clock::now(); // 종료 시간 기록
                std::chrono::duration<float> elapsed_seconds = end - start; // 경과 시간 계산
                total_frame++; // 처리한 프레임 수 증가
                float time_temp = elapsed_seconds.count() * 1000; // 경과 시간을 밀리초로 변환
                total_time += time_temp; // 총 시간 누적
                ROS_INFO("average odom estimation time %f ms \n \n", total_time / total_frame); // 평균 오도메트리 추정 시간 출력
            }

            // 현재 오도메트리 상태에서의 회전과 변환을 얻습니다.
            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            
            // q_current.normalize(); // 회전 값을 정규화 (사용하지 않음)
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            // TF 변환 브로드캐스터를 사용하여 좌표 변환을 전송합니다.
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z())); // 변환의 원점을 설정
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w()); // 변환의 회전 설정
            transform.setRotation(q); // 변환에 회전 적용
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link")); // TF 전송

            // 오도메트리 메시지를 생성하고 퍼블리시합니다.
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map"; // 부모 프레임 설정
            laserOdometry.child_frame_id = "base_link"; // 자식 프레임 설정
            laserOdometry.header.stamp = pointcloud_time; // 타임스탬프 설정
            laserOdometry.pose.pose.orientation.x = q_current.x(); // 회전 값 설정
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x(); // 위치 값 설정
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry); // 오도메트리 퍼블리시
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

    // Lidar 파라미터 설정
    int scan_line = 64; // 라이다 스캔 라인 수
    double vertical_angle = 2.0; // 라이다의 수직 각도
    double scan_period = 0.1; // 라이다 스캔 주기
    double max_dis = 60.0; // 라이다 최대 거리
    double min_dis = 2.0; // 라이다 최소 거리
    double map_resolution = 0.4; // 맵 해상도

    // ROS 파라미터 서버에서 파라미터를 읽어옵니다.
    nh.getParam("/scan_period", scan_period); // 스캔 주기 파라미터 가져오기
    nh.getParam("/vertical_angle", vertical_angle); // 수직 각도 파라미터 가져오기
    nh.getParam("/max_dis", max_dis); // 최대 거리 파라미터 가져오기
    nh.getParam("/min_dis", min_dis); // 최소 거리 파라미터 가져오기
    nh.getParam("/scan_line", scan_line); // 스캔 라인 수 파라미터 가져오기
    nh.getParam("/map_resolution", map_resolution); // 맵 해상도 파라미터 가져오기

    // Lidar 파라미터 설정
    lidar_param.setScanPeriod(scan_period); // 스캔 주기 설정
    lidar_param.setVerticalAngle(vertical_angle); // 수직 각도 설정
    lidar_param.setLines(scan_line); // 스캔 라인 수 설정
    lidar_param.setMaxDistance(max_dis); // 최대 거리 설정
    lidar_param.setMinDistance(min_dis); // 최소 거리 설정

    // 오도메트리 추정 클래스 초기화
    odomEstimation.init(lidar_param, map_resolution); // 라이다 파라미터와 맵 해상도로 초기화

    // 에지 포인트 클라우드와 서프 포인트 클라우드를 구독합니다.
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler); // 에지 포인트 클라우드 구독
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler); // 서프 포인트 클라우드 구독

    // 오도메트리를 퍼블리시할 퍼블리셔 설정
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    // 오도메트리 추정 프로세스를 실행하는 스레드를 생성합니다.
    std::thread odom_estimation_process{odom_estimation};

    // ROS 이벤트 루프를 실행하여 콜백을 처리합니다.
    ros::spin();

    return 0; // 프로그램 종료
}


