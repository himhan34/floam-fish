// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimization.h"

// EdgeAnalyticCostFunction 생성자, 현재 점(curr_point_), 마지막 점 A(last_point_a_), 마지막 점 B(last_point_b_)을 초기화
EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_){

}

// Evaluate 함수, Ceres Solver에서 호출되어 잔차(residuals)와 자코비안(jacobians)을 계산
bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // 쿼터니언으로 변환된 현재 위치와 회전(q_last_curr)을 얻고, 마지막 위치로의 변환(t_last_curr)을 맵핑
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    
    // 현재 점(curr_point)을 마지막 위치로 변환된 좌표(lp)로 계산
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; 

    // 두 벡터 (lp - last_point_a)와 (lp - last_point_b)의 외적을 계산하여 잔차 계산에 사용
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    
    // 마지막 두 점(last_point_a와 last_point_b) 사이의 벡터를 계산하고 그 크기(de_norm)를 구함
    Eigen::Vector3d de = last_point_a - last_point_b;
    double de_norm = de.norm();
    
    // 잔차(residuals)는 외적 벡터의 크기(nu.norm)를 마지막 두 점 사이의 거리로 나눈 값
    residuals[0] = nu.norm() / de_norm;
    
    // 자코비안(jacobians) 계산이 필요한 경우
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            // lp의 스큐 행렬(skew matrix)을 계산
            Eigen::Matrix3d skew_lp = skew(lp);
            
            // SE3에 대한 미분(dp_by_se3) 계산, 첫 3x3 블록은 -skew_lp, 마지막 블록은 단위 행렬로 설정
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_lp;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            
            // 자코비안을 맵핑하고 초기화
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            
            // 마지막 두 점 사이의 벡터의 스큐 행렬(skew_de)을 계산하고 자코비안 블록을 업데이트
            Eigen::Matrix3d skew_de = skew(de);
            J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
        }
    }  

    return true;
}   


SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_) 
                                                        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_){
    // 생성자: curr_point, plane_unit_norm, negative_OA_dot_norm 초기화
}

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // Evaluate 함수: 주어진 매개변수에 대한 잔차와 선택적 Jacobian 행렬을 계산
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    // 매개변수로부터 쿼터니언 q_w_curr 매핑
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    // 매개변수로부터 위치 벡터 t_w_curr 매핑
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
    // 현재 점을 회전하고 이동하여 월드 좌표계의 점을 계산
    residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;
    // 잔차 계산: 평면 법선 벡터와 점의 내적에 상수 값을 더함

    if(jacobians != NULL)
    {
        // Jacobian이 필요하다면
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);
            // 점의 스큐 대칭 행렬 생성
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            // SE(3)에 대한 점의 미분 행렬 선언
            dp_by_se3.block<3,3>(0,0) = -skew_point_w;
            // 회전에 대한 파트: 스큐 대칭 행렬로 설정
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            // 이동에 대한 파트: 단위 행렬로 설정
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            // Jacobian 매핑
            J_se3.setZero();
            // Jacobian을 0으로 초기화
            J_se3.block<1,6>(0,0) = plane_unit_norm.transpose() * dp_by_se3;
            // Jacobian 계산
        }
    }
    return true;
    // 함수 성공적으로 종료
}   

#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)
bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // Plus 함수: x와 delta를 합하여 새로운 x_plus_delta 계산
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);
    // 매개변수 x에서 위치 벡터 추출

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    // delta로부터 쿼터니언과 이동 벡터 추출
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);
    // x와 x_plus_delta를 쿼터니언과 위치 벡터로 매핑

    quater_plus = delta_q * quater;
    // 새로운 쿼터니언 계산
    trans_plus = delta_q * trans + delta_t;
    // 새로운 위치 벡터 계산
    return true;
}

bool PoseSE3Parameterization::Minus(const double *x, const double *delta, double *x_minus_delta) const
{
    // Minus 함수: x와 delta를 사용해 새로운 x_minus_delta 계산
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);
    // 매개변수 x에서 위치 벡터 추출

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    // delta로부터 쿼터니언과 이동 벡터 추출
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_minus(x_minus_delta);
    Eigen::Map<Eigen::Vector3d> trans_minus(x_minus_delta + 4);
    // x와 x_minus_delta를 쿼터니언과 위치 벡터로 매핑

    quater_minus = delta_q.inverse() * quater;
    // 새로운 쿼터니언 계산 (역변환 적용)
    trans_minus = delta_q.inverse() * trans - delta_t;
    // 새로운 위치 벡터 계산 (역변환 적용)
    return true;
}

bool PoseSE3Parameterization::PlusJacobian(const double *x, double *jacobian) const
{
    // PlusJacobian 함수: x에 대한 Jacobian 계산
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    // 상위 6개 행은 단위 행렬로 설정
    (j.bottomRows(1)).setZero();
    // 마지막 1개 행은 0으로 설정
    return true;
}

bool PoseSE3Parameterization::MinusJacobian(const double *x, double *jacobian) const
{
    // MinusJacobian 함수: x에 대한 Jacobian 계산
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    // 상위 6개 행은 단위 행렬로 설정
    (j.bottomRows(1)).setZero();
    // 마지막 1개 행은 0으로 설정
    return true;
}

#else

bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // Plus 함수: x와 delta를 합하여 새로운 x_plus_delta 계산
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);
    // 매개변수 x에서 위치 벡터 추출

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    // delta로부터 쿼터니언과 이동 벡터 추출
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);
    // x와 x_plus_delta를 쿼터니언과 위치 벡터로 매핑

    quater_plus = delta_q * quater;
    // 새로운 쿼터니언 계산
    trans_plus = delta_q * trans + delta_t;
    // 새로운 위치 벡터 계산
    return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    // ComputeJacobian 함수: x에 대한 Jacobian 계산
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    // 상위 6개 행은 단위 행렬로 설정
    (j.bottomRows(1)).setZero();
    // 마지막 1개 행은 0으로 설정
    return true;
}

#endif

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t){
    // getTransformFromSe3 함수: se3 형식으로부터 쿼터니언 q와 이동 벡터 t 추출
    Eigen::Vector3d omega(se3.data());
    // se3로부터 회전 벡터 omega 추출
    Eigen::Vector3d upsilon(se3.data()+3);
    // se3로부터 이동 벡터 upsilon 추출
    Eigen::Matrix3d Omega = skew(omega);
    // omega로부터 스큐 대칭 행렬 Omega 생성

    double theta = omega.norm();
    // 회전 벡터의 크기 계산
    double half_theta = 0.5*theta;
    // 회전 각도의 절반 계산

    double imag_factor;
    double real_factor = cos(half_theta);
    // 쿼터니언의 실수부 계산
    if(theta<1e-10)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
        // theta가 매우 작은 경우에 대한 근사값 계산
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
        // 일반적인 경우의 허수부 계산
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());
    // 쿼터니언 생성

    Eigen::Matrix3d J;
    if (theta<1e-10)
    {
        J = q.matrix();
        // theta가 작은 경우 J는 쿼터니언 행렬로 설정
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
        // 일반적인 경우 J 행렬 계산
    }

    t = J*upsilon;
    // 이동 벡터 t 계산
}

Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){
    // skew 함수: 입력 벡터로부터 스큐 대칭 행렬 생성
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    // 스큐 행렬을 0으로 초기화
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    // 스큐 행렬의 요소들을 설정
    return skew_mat;
    // 생성된 스큐 행렬 반환
}
