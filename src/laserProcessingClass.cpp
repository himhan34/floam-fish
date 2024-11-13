// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserProcessingClass.h"

// LaserProcessingClass의 초기화 함수: 라이다 파라미터 설정
void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    // 입력받은 라이다 파라미터를 멤버 변수에 저장
    lidar_param = lidar_param_in;

}

// 특징 추출 함수: 입력된 포인트 클라우드에서 에지와 서프 포인트를 추출
void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    // 포인트 클라우드에서 NaN값 제거
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    // 라이다 스캔 수 설정
    int N_SCANS = lidar_param.num_lines;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    // 각 스캔 라인에 대해 포인트 클라우드 객체 생성
    for(int i=0;i<N_SCANS;i++){
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
    }

    // 입력된 포인트 클라우드의 모든 포인트에 대해 반복
    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        int scanID=0;
        // 포인트의 거리 계산 (x, y 좌표 사용)
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
        // 최소 및 최대 거리 기준에 맞지 않으면 해당 포인트는 건너뜀
        if(distance<lidar_param.min_distance || distance>lidar_param.max_distance)
            continue;
        // 포인트의 각도 계산 (세로 방향)
        double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
        
        // 스캔 수에 따른 스캔 ID 결정
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
            {
                continue;
            }
        }
        else
        {
            // 잘못된 스캔 수가 입력된 경우 오류 메시지 출력
            printf("wrong scan number\n");
        }
        // 해당 스캔 라인에 포인트 추가
        laserCloudScans[scanID]->push_back(pc_in->points[i]); 

    }

    // 각 스캔 라인에 대해 특징 추출 진행
    for(int i = 0; i < N_SCANS; i++){
        // 포인트 개수가 너무 적으면 건너뜀
        if(laserCloudScans[i]->points.size()<131){
            continue;
        }
        
        // 포인트들의 곡률 값을 저장하기 위한 벡터
        std::vector<Double2d> cloudCurvature; 
        int total_points = laserCloudScans[i]->points.size()-10;
        // 곡률 계산을 위해 주변 10개의 포인트와 비교
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }
        // 각 스캔 라인을 6개의 섹터로 나누어 특징 추출 수행
        for(int j=0;j<6;j++){
            int sector_length = (int)(total_points/6);
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = total_points - 1; 
            }
            // 섹터에 해당하는 곡률 값만을 추출
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end); 
            
            // 섹터에서 특징 추출 (에지 및 서프 포인트)
            featureExtractionFromSector(laserCloudScans[i],subCloudCurvature, pc_out_edge, pc_out_surf);
            
        }

    }

} 


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    // cloudCurvature를 value 기준으로 오름차순 정렬
    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
    { 
        return a.value < b.value; 
    });

    // 가장 큰 curvature를 가진 점들을 선택하기 위한 변수 초기화
    int largestPickedNum = 0;
    std::vector<int> picked_points; // 선택된 점들의 인덱스를 저장할 벡터
    int point_info_count =0; // 엣지 점 개수 카운트

    // cloudCurvature 벡터를 역순으로 순회
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; // 현재 점의 인덱스 가져오기
        
        // 현재 점이 이미 선택된 점에 포함되지 않았다면
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            // curvature 값이 0.1 이하이면 종료
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            // 선택된 점 개수 증가
            largestPickedNum++;
            picked_points.push_back(ind); // 현재 점 인덱스를 선택된 점 목록에 추가
            
            // 20개 이하의 점이라면 엣지 포인트로 추가
            if (largestPickedNum <= 20){
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break; // 20개 이상의 점을 선택한 경우 종료
            }

            // 현재 점의 다음 5개 점에 대해 유사한 점들을 선택 목록에 추가
            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x; // x 좌표 차이 계산
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y; // y 좌표 차이 계산
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z; // z 좌표 차이 계산
                
                // 좌표 차이의 제곱합이 0.05보다 크면 반복 종료
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k); // 선택된 점 목록에 추가
            }

            // 현재 점의 이전 5개 점에 대해 유사한 점들을 선택 목록에 추가
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x; // x 좌표 차이 계산
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y; // y 좌표 차이 계산
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z; // z 좌표 차이 계산
                
                // 좌표 차이의 제곱합이 0.05보다 크면 반복 종료
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k); // 선택된 점 목록에 추가
            }
        }
    }
    
/*
        // 평탄한 점들을 찾기 위한 코드 시작
    // point_info_count 초기화
    point_info_count =0;
    // 가장 작은 점들을 저장할 변수 초기화
    int smallestPickedNum = 0;
    
    // cloudCurvature 배열 전체를 순회
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        // 현재 처리 중인 점의 인덱스 가져오기
        int ind = cloudCurvature[i].id; 

        // 이미 선택된 점 목록에 포함되어 있는지 확인
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            // 곡률 값이 0.1보다 크면, LiDAR 상태에 문제가 있을 수 있으므로 경고 메시지를 출력하고 종료
            if(cloudCurvature[i].value > 0.1){
                //ROS_WARN("extracted feature not qualified, please check lidar");
                break;
            }
            // 가장 작은 점 개수 증가
            smallestPickedNum++;
            // 현재 점을 선택된 점 목록에 추가
            picked_points.push_back(ind);
            
            // 4개 이하의 점이라면 평탄한 점 목록에 추가
            if(smallestPickedNum <= 4){
                // 모든 점 찾기
                pc_surf_flat->push_back(pc_in->points[ind]);
                pc_surf_lessFlat->push_back(pc_in->points[ind]);
                point_info_count++;
            }
            else{
                break;
            }

            // 인접한 5개의 점 검사 (앞 방향)
            for(int k=1;k<=5;k++){
                // 인접한 점들 간의 차이 계산
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                // 점 간의 차이가 0.05를 넘으면 중단
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                // 인덱스를 선택된 점 목록에 추가
                picked_points.push_back(ind+k);
            }
            // 인접한 5개의 점 검사 (뒤 방향)
            for(int k=-1;k>=-5;k--){
                // 인접한 점들 간의 차이 계산
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                // 점 간의 차이가 0.05를 넘으면 중단
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                // 인덱스를 선택된 점 목록에 추가
                picked_points.push_back(ind+k);
            }

        }
    }
*/


  
// 포인트 클라우드에서 각 점의 곡률 정보를 바탕으로 필터링하는 루프
for (int i = 0; i <= (int)cloudCurvature.size() - 1; i++)
{
    // cloudCurvature의 인덱스를 가져옴
    int ind = cloudCurvature[i].id;
    // 이미 선택된 점인지 확인
    if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end())
    {
        // 선택되지 않은 점이라면 출력 포인트 클라우드에 추가
        pc_out_surf->push_back(pc_in->points[ind]);
    }
}

// LaserProcessingClass 생성자 정의
LaserProcessingClass::LaserProcessingClass() {
    // 특별한 초기화는 없음
}

// Double2d 클래스의 생성자 정의
Double2d::Double2d(int id_in, double value_in) {
    // id와 value 멤버 변수 초기화
    id = id_in;
    value = value_in;
}

// PointsInfo 클래스의 생성자 정의
PointsInfo::PointsInfo(int layer_in, double time_in) {
    // layer와 time 멤버 변수 초기화
    layer = layer_in;
    time = time_in;
} 

