// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// C++ 라이브러리
#include <cmath> // 수학 함수들을 사용하기 위한 라이브러리
#include <vector> // 벡터 자료 구조를 사용하기 위한 라이브러리
#include <mutex> // 스레드 간 동기화를 위한 뮤텍스 라이브러리
#include <queue> // 큐 자료 구조를 사용하기 위한 라이브러리
#include <thread> // 멀티스레딩을 위한 라이브러리
#include <chrono> // 시간 관련 기능을 사용하기 위한 라이브러리

// ROS 라이브러리
#include <ros/ros.h> // ROS의 기본 기능을 사용하기 위한 라이브러리
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 메시지를 사용하기 위한 라이브러리
#include <nav_msgs/Odometry.h> // 로봇의 위치와 속도를 다루는 메시지를 사용하기 위한 라이브러리
#include <tf/transform_datatypes.h> // 변환 데이터 타입을 사용하기 위한 라이브러리
#include <tf/transform_broadcaster.h> // 좌표계 변환을 브로드캐스팅하기 위한 라이브러리

// PCL (Point Cloud Library) 라이브러리
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 간의 포인트 클라우드 데이터 변환을 위한 라이브러리
#include <pcl/point_cloud.h> // 포인트 클라우드 데이터 구조를 사용하기 위한 라이브러리
#include <pcl/point_types.h> // 다양한 포인트 타입을 정의하는 라이브러리

// 로컬 라이브러리
#include "laserMappingClass.h" // 레이저 맵핑 클래스를 포함하는 로컬 헤더 파일
#include "lidar.h" // 라이다 관련 기능을 포함하는 로컬 헤더 파일

// LaserMappingClass의 객체 생성
LaserMappingClass laserMapping;

// lidar 네임스페이스의 Lidar 클래스 객체 생성
lidar::Lidar lidar_param;

// 스레드 간 동기화를 위한 뮤텍스 객체 생성
std::mutex mutex_lock;

// nav_msgs::OdometryConstPtr 타입의 메시지를 저장하는 큐 생성
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;

// sensor_msgs::PointCloud2ConstPtr 타입의 메시지를 저장하는 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

// ROS 퍼블리셔 객체 생성 (맵 데이터를 퍼블리시하기 위한 객체)
ros::Publisher map_pub;

// Odometry 메시지를 수신할 때 호출되는 콜백 함수
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 뮤텍스를 잠가서 스레드 간 데이터 충돌 방지
    mutex_lock.lock();
    // 수신된 메시지를 odometryBuf 큐에 추가
    odometryBuf.push(msg);
    // 뮤텍스를 해제
    mutex_lock.unlock();
}

// Velodyne 포인트 클라우드 메시지를 수신할 때 호출되는 콜백 함수
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // 뮤텍스를 잠가서 스레드 간 데이터 충돌 방지
    mutex_lock.lock();
    // 수신된 포인트 클라우드 메시지를 pointCloudBuf 큐에 추가
    pointCloudBuf.push(laserCloudMsg);
    // 뮤텍스를 해제
    mutex_lock.unlock();
}
// 여기서 mid-360 handler 만들 수 있을 것 같음. 



// Laser mapping 작업을 수행하는 함수
void laser_mapping() {
    while (1) {
        // odometryBuf와 pointCloudBuf 둘 다 비어 있지 않은 경우
        if (!odometryBuf.empty() && !pointCloudBuf.empty()) {

            // 데이터 읽기
            mutex_lock.lock();
            // pointCloudBuf의 시간 스탬프가 odometryBuf의 시간 스탬프와 정렬되지 않은 경우
            if (!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period) {
                // 시간 정렬 오류 경고 메시지를 출력하고 포인트 클라우드 데이터 삭제
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node");
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            // odometryBuf의 시간 스탬프가 pointCloudBuf의 시간 스탬프와 정렬되지 않은 경우
            if (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period) {
                // 경고 메시지를 출력하고 오도메트리 데이터 삭제
                odometryBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            // 시간이 정렬된 경우
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in); // 포인트 클라우드 데이터를 변환
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp; // 포인트 클라우드 시간 스탬프 가져오기

            // 오도메트리 데이터를 기반으로 현재 자세를 설정
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(
                odometryBuf.front()->pose.pose.orientation.w,
                odometryBuf.front()->pose.pose.orientation.x,
                odometryBuf.front()->pose.pose.orientation.y,
                odometryBuf.front()->pose.pose.orientation.z
            ));  
            current_pose.pretranslate(Eigen::Vector3d(
                odometryBuf.front()->pose.pose.position.x,
                odometryBuf.front()->pose.pose.position.y,
                odometryBuf.front()->pose.pose.position.z
            ));
            
            // 큐에서 사용한 데이터를 삭제
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();

            // 현재 포인트를 맵에 업데이트
            laserMapping.updateCurrentPointsToMap(pointcloud_in, current_pose);

            // 맵 데이터를 가져와서 ROS 메시지로 변환 후 퍼블리시
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*pc_map, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "map";
            map_pub.publish(PointsMsg);

        }
        // 매 루프마다 2ms 동안 슬립
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // 기본 라이다 파라미터 설정
    int scan_line = 64; // 스캔 라인의 개수
    double vertical_angle = 2.0; // 라이다의 수직 각도
    double scan_period = 0.1; // 스캔 주기
    double max_dis = 60.0; // 최대 거리
    double min_dis = 2.0; // 최소 거리
    double map_resolution = 0.4; // 맵 해상도

    // ROS 파라미터 서버에서 파라미터를 가져옴
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    // 라이다 파라미터 설정
    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    // LaserMappingClass 초기화
    laserMapping.init(map_resolution);

    // 포인트 클라우드와 오도메트리 데이터를 수신하기 위한 ROS 구독자 설정
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    // 맵 데이터를 퍼블리시하기 위한 ROS 퍼블리셔 설정
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);

    // 레이저 매핑 작업을 별도의 스레드로 실행
    std::thread laser_mapping_process{laser_mapping};

    // ROS 이벤트 루프 실행
    ros::spin();

    return 0; // 프로그램 종료
}

