// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingClass.h"

void LaserMappingClass::init(double map_resolution){
	// 맵 초기화
	// 초기화 시 맵에 실제 객체를 추가할 수 있지만, 이후에 추가될 블록은 필요 없음
	for(int i=0; i<LASER_CELL_RANGE_HORIZONTAL*2+1; i++){  // 가로 범위만큼 반복
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;  // 맵의 높이를 임시로 저장할 벡터 생성
		for(int j=0; j<LASER_CELL_RANGE_HORIZONTAL*2+1; j++){  // 세로 범위만큼 반복
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;  // 맵의 깊이를 임시로 저장할 벡터 생성
			for(int k=0; k<LASER_CELL_RANGE_VERTICAL*2+1; k++){  // 수직 범위만큼 반복
				pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);  // 새로운 점군 객체 생성
				map_depth_temp.push_back(point_cloud_temp);  // 깊이 벡터에 점군 객체 추가
			}
			map_height_temp.push_back(map_depth_temp);  // 높이 벡터에 깊이 벡터 추가
		}
		map.push_back(map_height_temp);  // 전체 맵에 높이 벡터 추가
	}

	origin_in_map_x = LASER_CELL_RANGE_HORIZONTAL;  // 맵의 원점 x 좌표 설정
	origin_in_map_y = LASER_CELL_RANGE_HORIZONTAL;  // 맵의 원점 y 좌표 설정
	origin_in_map_z = LASER_CELL_RANGE_VERTICAL;    // 맵의 원점 z 좌표 설정
	map_width = LASER_CELL_RANGE_HORIZONTAL*2+1;    // 맵의 가로 크기 설정
	map_height = LASER_CELL_RANGE_HORIZONTAL*2+1;   // 맵의 세로 크기 설정
	map_depth = LASER_CELL_RANGE_HORIZONTAL*2+1;    // 맵의 깊이 크기 설정

	// 다운샘플링 필터 크기 설정
	downSizeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);  // 다운샘플링 필터의 해상도를 설정
}

void LaserMappingClass::addWidthCellNegative(void){
    // 너비 방향 음의 방향으로 셀을 추가하는 함수
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
    // 임시 높이 벡터(map_height_temp)를 선언
    for(int j=0; j < map_height;j++){
        // 높이(map_height)만큼 반복
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
        // 임시 깊이 벡터(map_depth_temp)를 선언
        for(int k=0;k< map_depth;k++){
            // 깊이(map_depth)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map_depth_temp.push_back(point_cloud_temp);
            // 깊이 벡터에 포인트 클라우드 포인터를 추가
        }
        map_height_temp.push_back(map_depth_temp);
        // 높이 벡터에 깊이 벡터를 추가
    }
    map.insert(map.begin(), map_height_temp);
    // 맵(map)의 시작 부분에 높이 벡터를 삽입하여 너비 방향 음의 방향에 셀을 추가

    origin_in_map_x++;
    // 맵에서의 원점 x 좌표를 증가
    map_width++;
    // 맵의 너비를 증가
}

void LaserMappingClass::addWidthCellPositive(void){
    // 너비 방향 양의 방향으로 셀을 추가하는 함수
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
    // 임시 높이 벡터(map_height_temp)를 선언
    for(int j=0; j < map_height;j++){
        // 높이(map_height)만큼 반복
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
        // 임시 깊이 벡터(map_depth_temp)를 선언
        for(int k=0;k< map_depth;k++){
            // 깊이(map_depth)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map_depth_temp.push_back(point_cloud_temp);
            // 깊이 벡터에 포인트 클라우드 포인터를 추가
        }
        map_height_temp.push_back(map_depth_temp);
        // 높이 벡터에 깊이 벡터를 추가
    }
    map.push_back(map_height_temp);
    // 맵(map)의 끝 부분에 높이 벡터를 추가하여 너비 방향 양의 방향에 셀을 추가
    map_width++;
    // 맵의 너비를 증가
}

void LaserMappingClass::addHeightCellNegative(void){
    // 높이 방향 음의 방향으로 셀을 추가하는 함수
    for(int i=0; i < map_width;i++){
        // 너비(map_width)만큼 반복
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
        // 임시 깊이 벡터(map_depth_temp)를 선언
        for(int k=0;k<map_depth;k++){
            // 깊이(map_depth)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map_depth_temp.push_back(point_cloud_temp);
            // 깊이 벡터에 포인트 클라우드 포인터를 추가
        }
        map[i].insert(map[i].begin(), map_depth_temp);
        // 각 너비 위치에서 높이 벡터의 시작 부분에 깊이 벡터를 삽입
    }
    origin_in_map_y++;
    // 맵에서의 원점 y 좌표를 증가
    map_height++;
    // 맵의 높이를 증가
}

void LaserMappingClass::addHeightCellPositive(void){
    // 높이 방향 양의 방향으로 셀을 추가하는 함수
    for(int i=0; i < map_width;i++){
        // 너비(map_width)만큼 반복
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
        // 임시 깊이 벡터(map_depth_temp)를 선언
        for(int k=0;k<map_depth;k++){
            // 깊이(map_depth)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map_depth_temp.push_back(point_cloud_temp);
            // 깊이 벡터에 포인트 클라우드 포인터를 추가
        }
        map[i].push_back(map_depth_temp);
        // 각 너비 위치에서 높이 벡터의 끝 부분에 깊이 벡터를 추가
    }
    map_height++;
    // 맵의 높이를 증가
}

void LaserMappingClass::addDepthCellNegative(void){
    // 깊이 방향 음의 방향으로 셀을 추가하는 함수
    for(int i=0; i < map_width;i++){
        // 너비(map_width)만큼 반복
        for(int j=0;j< map_height;j++){
            // 높이(map_height)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map[i][j].insert(map[i][j].begin(), point_cloud_temp);
            // 각 위치에서 깊이 벡터의 시작 부분에 포인트 클라우드 포인터를 삽입
        }
    }
    origin_in_map_z++;
    // 맵에서의 원점 z 좌표를 증가
    map_depth++;
    // 맵의 깊이를 증가
}

void LaserMappingClass::addDepthCellPositive(void){
    // 깊이 방향 양의 방향으로 셀을 추가하는 함수
    for(int i=0; i < map_width;i++){
        // 너비(map_width)만큼 반복
        for(int j=0;j< map_height;j++){
            // 높이(map_height)만큼 반복
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
            // 포인트 클라우드 포인터(point_cloud_temp)를 선언
            map[i][j].push_back(point_cloud_temp);
            // 각 위치에서 깊이 벡터의 끝 부분에 포인트 클라우드 포인터를 추가
        }
    }
    map_depth++;
    // 맵의 깊이를 증가
}

// 포인트가 맵의 크기를 초과하면 맵을 확장하는 함수
void LaserMappingClass::checkPoints(int& x, int& y, int& z){

    while(x + LASER_CELL_RANGE_HORIZONTAL > map_width - 1){
        // x 좌표가 맵의 너비를 초과하면 양의 방향으로 셀을 추가
        addWidthCellPositive();
    }
    while(x - LASER_CELL_RANGE_HORIZONTAL < 0){
        // x 좌표가 음수이면 음의 방향으로 셀을 추가하고 x를 보정
        addWidthCellNegative();
        x++;
    }
    while(y + LASER_CELL_RANGE_HORIZONTAL > map_height - 1){
        // y 좌표가 맵의 높이를 초과하면 양의 방향으로 셀을 추가
        addHeightCellPositive();
    }
    while(y - LASER_CELL_RANGE_HORIZONTAL < 0){
        // y 좌표가 음수이면 음의 방향으로 셀을 추가하고 y를 보정
        addHeightCellNegative();
        y++;
    }
    while(z + LASER_CELL_RANGE_VERTICAL > map_depth - 1){
        // z 좌표가 맵의 깊이를 초과하면 양의 방향으로 셀을 추가
        addDepthCellPositive();
    }
    while(z - LASER_CELL_RANGE_VERTICAL < 0){
        // z 좌표가 음수이면 음의 방향으로 셀을 추가하고 z를 보정
        addDepthCellNegative();
        z++;
    }

    // 초기화
    // 해당 영역이 비어 있으면 객체를 생성
    for(int i = x - LASER_CELL_RANGE_HORIZONTAL; i < x + LASER_CELL_RANGE_HORIZONTAL + 1; i++){
        for(int j = y - LASER_CELL_RANGE_HORIZONTAL; j < y + LASER_CELL_RANGE_HORIZONTAL + 1; j++){
            for(int k = z - LASER_CELL_RANGE_VERTICAL; k < z + LASER_CELL_RANGE_VERTICAL + 1; k++){
                if(map[i][j][k] == NULL){
                    // 새로운 포인트 클라우드 객체를 생성하여 맵에 할당
                    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
                    map[i][j][k] = point_cloud_temp;
                }
            }
        }
    }
}

// 현재의 포인트들을 맵에 업데이트하는 함수
void LaserMappingClass::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current){
    
    int currentPosIdX = int(std::floor(pose_current.translation().x() / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
    // 현재 위치의 x 인덱스를 계산
    int currentPosIdY = int(std::floor(pose_current.translation().y() / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
    // 현재 위치의 y 인덱스를 계산
    int currentPosIdZ = int(std::floor(pose_current.translation().z() / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;
    // 현재 위치의 z 인덱스를 계산

    // 서브맵이 비어 있는지 확인하고 필요한 경우 확장
    checkPoints(currentPosIdX, currentPosIdY, currentPosIdZ);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
    // 입력 포인트 클라우드를 현재 위치로 변환
    pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());
    
    // 포인트를 저장
    for (int i = 0; i < (int)transformed_pc->points.size(); i++)
    {
        pcl::PointXYZI point_temp = transformed_pc->points[i];
        // 시각화를 위해 intensity 값을 조정
        point_temp.intensity = std::min(1.0, std::max(pc_in->points[i].z + 2.0, 0.0) / 5);
        int currentPointIdX = int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
        // 포인트의 x 인덱스를 계산
        int currentPointIdY = int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
        // 포인트의 y 인덱스를 계산
        int currentPointIdZ = int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;
        // 포인트의 z 인덱스를 계산

        map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
        // 해당 위치의 맵에 포인트를 추가
    }
    
    // 포인트를 필터링
    for(int i = currentPosIdX - LASER_CELL_RANGE_HORIZONTAL; i < currentPosIdX + LASER_CELL_RANGE_HORIZONTAL + 1; i++){
        for(int j = currentPosIdY - LASER_CELL_RANGE_HORIZONTAL; j < currentPosIdY + LASER_CELL_RANGE_HORIZONTAL + 1; j++){
            for(int k = currentPosIdZ - LASER_CELL_RANGE_VERTICAL; k < currentPosIdZ + LASER_CELL_RANGE_VERTICAL + 1; k++){
                downSizeFilter.setInputCloud(map[i][j][k]);
                // 다운샘플링 필터를 적용
                downSizeFilter.filter(*(map[i][j][k]));
            }
        }
    }

}

// 맵을 가져오는 함수
pcl::PointCloud<pcl::PointXYZI>::Ptr LaserMappingClass::getMap(void){
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap(new pcl::PointCloud<pcl::PointXYZI>());
    // 전체 맵을 순회하며 포인트를 수집
    for (int i = 0; i < map_width; i++){
        for (int j = 0; j < map_height; j++){
            for (int k = 0; k < map_depth; k++){
                if(map[i][j][k] != NULL){
                    *laserCloudMap += *(map[i][j][k]);
                    // 맵의 포인트들을 laserCloudMap에 추가
                }
            }
        }
    }
    return laserCloudMap;
    // 전체 맵을 반환
}

// 클래스의 생성자
LaserMappingClass::LaserMappingClass(){

}

