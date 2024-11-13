// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution){
    // 로컬 맵 초기화
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    // 다운샘플링 크기 설정
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    // kd-트리 초기화
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    // 초기 위치 추정 행렬을 항등 행렬로 설정
    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();

    // 초기 최적화 횟수 설정
    optimization_count = 2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    // 맵에 코너 및 평면 포인트 추가
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    
    // 최적화 횟수 증가
    optimization_count = 12;
}

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    // optimization_count가 2보다 크면 1 감소시킵니다.
    if(optimization_count>2)
        optimization_count--;

    // 이전 odom을 사용하여 현재 odom을 예측합니다.
    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    // 마지막 odom을 현재 odom으로 업데이트합니다.
    last_odom = odom;
    // 현재 odom을 예측된 odom으로 설정합니다.
    odom = odom_prediction;

    // 현재 odom의 회전을 쿼터니언 형태로 변환합니다.
    q_w_curr = Eigen::Quaterniond(odom.rotation());
    // 현재 odom의 평행 이동 벡터를 가져옵니다.
    t_w_curr = odom.translation();

    // 에지와 표면 포인트 클라우드를 다운샘플링하기 위한 포인터를 생성합니다.
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    // 입력 포인트 클라우드를 맵에 맞게 다운샘플링합니다.
    downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in, downsampledSurfCloud);
    // ROS 경고 메시지를 출력합니다. (포인트 개수 확인)
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());

    // 맵에 충분한 에지와 표면 포인트가 있는지 확인합니다.
    if(laserCloudCornerMap->points.size() > 10 && laserCloudSurfMap->points.size() > 50){
        // kdtree에 에지와 표면 맵 포인트 클라우드를 설정합니다.
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        // 최적화 반복 횟수만큼 루프를 실행합니다.
        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            // Huber 손실 함수를 생성합니다.
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            // Ceres 최적화 문제 옵션을 설정합니다.
            ceres::Problem::Options problem_options;
            // Ceres 최적화 문제를 생성합니다.
            ceres::Problem problem(problem_options);

            // 최적화 문제에 파라미터 블록을 추가합니다.
            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            // 에지와 표면 비용 요소를 최적화 문제에 추가합니다.
            addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap, problem, loss_function);
            addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, problem, loss_function);

            // Ceres Solver 옵션을 설정합니다.
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            // Ceres Solver를 실행하여 최적화를 수행합니다.
            ceres::Solve(options, &problem, &summary);
        }
    }else{
        // 맵에 연관시킬 충분한 포인트가 없을 때 오류 메시지를 출력합니다.
        printf("not enough points in map to associate, map error");
    }

    // odom을 초기화하고 회전과 평행 이동을 업데이트합니다.
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    // 다운샘플링된 에지와 표면 포인트를 맵에 추가합니다.
    addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    // 현재 포인트를 Eigen 벡터로 변환합니다.
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    // 쿼터니언과 평행 이동 벡터를 사용하여 맵 좌표계로 변환합니다.
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    // 변환된 좌표를 결과 포인트에 할당합니다.
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    // 인텐시티 값을 그대로 유지합니다.
    po->intensity = pi->intensity;
    //po->intensity = 1.0; // 인텐시티를 1.0으로 고정할 수 있는 코드 (주석 처리됨)
}

// downSamplingToMap 함수는 edge 및 surface 포인트 클라우드를 다운샘플링하여 맵에 추가하는 역할을 합니다.
void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in); // edge 포인트 클라우드를 다운샘플링 필터에 설정
    downSizeFilterEdge.filter(*edge_pc_out); // 필터링된 edge 포인트 클라우드를 출력
    downSizeFilterSurf.setInputCloud(surf_pc_in); // surface 포인트 클라우드를 다운샘플링 필터에 설정
    downSizeFilterSurf.filter(*surf_pc_out); // 필터링된 surface 포인트 클라우드를 출력
}

// addEdgeCostFactor 함수는 edge 포인트 클라우드를 맵과 비교하여 최적화 문제의 코스트 팩터를 추가합니다.
void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 입력 포인트를 맵 좌표계로 변환

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); // K-최근접 이웃 탐색 (가장 가까운 5개의 점을 탐색)
        if (pointSearchSqDis[4] < 1.0) // 가장 먼 5번째 이웃 점의 거리가 1.0 미만인 경우에만 유효
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                // 주변 코너 점들의 좌표를 가져와 중심 좌표 계산
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0; // 5개의 점의 평균을 계산하여 중심 좌표 설정

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero(); // 공분산 행렬 초기화
            for (int j = 0; j < 5; j++)
            {
                // 각 점에서 중심을 뺀 후, 공분산 행렬을 계산
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat); // 고유값 분해를 통해 주성분을 계산

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 가장 큰 고유값에 대응하는 고유벡터를 방향 벡터로 사용
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) // 가장 큰 고유값이 두 번째로 큰 고유값의 3배 이상인 경우
            { 
                // 선상의 두 점을 계산하여 코스트 함수 생성에 사용
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line; // 중심에서 방향 벡터를 따라 0.1만큼 떨어진 점
                point_b = -0.1 * unit_direction + point_on_line; // 반대 방향으로 0.1만큼 떨어진 점

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  // Ceres 코스트 함수 생성
                problem.AddResidualBlock(cost_function, loss_function, parameters); // 최적화 문제에 잔차 블록 추가
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        printf("not enough correct points"); // 충분한 수의 유효한 코너 점이 없을 경우 경고 메시지 출력
    }
}

// addSurfCostFactor 함수는 surface 포인트 클라우드를 맵과 비교하여 최적화 문제의 코스트 팩터를 추가합니다.
void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 입력 포인트를 맵 좌표계로 변환
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); // K-최근접 이웃 탐색 (가장 가까운 5개의 점을 탐색)

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones(); // 평면 방정식을 위한 B 행렬 (-1로 초기화)
        if (pointSearchSqDis[4] < 1.0) // 가장 먼 5번째 이웃 점의 거리가 1.0 미만인 경우에만 유효
        {
            for (int j = 0; j < 5; j++)
            {
                // 주변 점들의 좌표를 A 행렬에 설정
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // 평면의 법선 벡터 계산 (AX + B = 0 형태에서 X를 구함)
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); // 최소제곱법을 이용해 법선 벡터 계산
            double negative_OA_dot_norm = 1 / norm.norm(); // 법선 벡터의 크기의 역수 계산
            norm.normalize(); // 법선 벡터를 정규화

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // OX * n > 0.2인 경우, 평면이 잘 맞지 않음
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false; // 평면이 유효하지 않음
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    // Ceres 코스트 함수 생성
                problem.AddResidualBlock(cost_function, loss_function, parameters); // 최적화 문제에 잔차 블록 추가
                surf_num++;
            }
        }
    }
    if(surf_num<20){
        printf("not enough correct points"); // 충분한 수의 유효한 서페이스 점이 없을 경우 경고 메시지 출력
    }
}

// 이 함수는 다운샘플링된 에지 및 서프 포인트 클라우드를 맵에 추가하고, 바운딩 박스를 사용하여 필터링한 후 다운샘플링합니다.
void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){

    // 다운샘플링된 에지 포인트 클라우드를 맵에 추가하는 루프
    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp); // 포인트를 맵 좌표계로 변환합니다.
        laserCloudCornerMap->push_back(point_temp); // 변환된 포인트를 에지 맵에 추가합니다.
    }
    
    // 다운샘플링된 서프 포인트 클라우드를 맵에 추가하는 루프
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp); // 포인트를 맵 좌표계로 변환합니다.
        laserCloudSurfMap->push_back(point_temp); // 변환된 포인트를 서프 맵에 추가합니다.
    }
    
    // 바운딩 박스의 최소값과 최대값을 설정하여 현재 위치를 중심으로 100m 범위를 정의합니다.
    double x_min = +odom.translation().x() - 100;
    double y_min = +odom.translation().y() - 100;
    double z_min = +odom.translation().z() - 100;
    double x_max = +odom.translation().x() + 100;
    double y_max = +odom.translation().y() + 100;
    double z_max = +odom.translation().z() + 100;
    
    // 바운딩 박스 필터의 최소값을 설정합니다.
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    // 바운딩 박스 필터의 최대값을 설정합니다.
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    // 필터가 바운딩 박스 내부의 포인트만 남기도록 설정합니다.
    cropBoxFilter.setNegative(false);    

    // 임시로 필터링된 에지 포인트와 서프 포인트 클라우드를 저장할 포인터를 생성합니다.
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap); // 서프 맵을 필터에 입력으로 설정합니다.
    cropBoxFilter.filter(*tmpSurf); // 서프 포인트 클라우드를 필터링합니다.
    cropBoxFilter.setInputCloud(laserCloudCornerMap); // 에지 맵을 필터에 입력으로 설정합니다.
    cropBoxFilter.filter(*tmpCorner); // 에지 포인트 클라우드를 필터링합니다.

    // 필터링된 서프 포인트 클라우드를 다운샘플링하여 맵에 저장합니다.
    downSizeFilterSurf.setInputCloud(tmpSurf); // 서프 포인트 클라우드를 다운샘플링 필터에 입력으로 설정합니다.
    downSizeFilterSurf.filter(*laserCloudSurfMap); // 다운샘플링된 서프 맵을 결과로 저장합니다.
    // 필터링된 에지 포인트 클라우드를 다운샘플링하여 맵에 저장합니다.
    downSizeFilterEdge.setInputCloud(tmpCorner); // 에지 포인트 클라우드를 다운샘플링 필터에 입력으로 설정합니다.
    downSizeFilterEdge.filter(*laserCloudCornerMap); // 다운샘플링된 에지 맵을 결과로 저장합니다.
}

// 이 함수는 서프 맵과 에지 맵을 합쳐서 전체 맵을 생성합니다.
void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap){
    
    // 서프 맵과 에지 맵을 합쳐 전체 맵을 생성합니다.
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

// 생성자: 초기화 작업은 필요하지 않습니다.
OdomEstimationClass::OdomEstimationClass(){

}



