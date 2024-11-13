// FLOAM의 작성자: Wang Han
// 이메일: wh200720041@gmail.com
// 홈페이지: https://wanghan.pro
#ifndef _LIDAR_OPTIMIZATION_ANALYTIC_H_  // _LIDAR_OPTIMIZATION_ANALYTIC_H_가 정의되지 않은 경우에만 이 코드를 실행합니다.
#define _LIDAR_OPTIMIZATION_ANALYTIC_H_  // _LIDAR_OPTIMIZATION_ANALYTIC_H_를 정의하여 중복 포함을 방지합니다.

#include <ceres/ceres.h>  // Ceres Solver의 기본 헤더 파일 포함
#include <ceres/rotation.h>  // Ceres 회전 관련 함수 포함
// 여기에서 업데이트
#include <ceres/version.h>  // Ceres 버전 정보를 가져오기 위한 헤더 포함
#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)  // Ceres 버전이 2.1 이상이거나 3 이상일 때
#include <ceres/manifold.h>  // Ceres의 Manifold 헤더 포함
#endif
#include <Eigen/Dense>  // Eigen 라이브러리의 밀집 행렬 관련 기능 포함
#include <Eigen/Geometry>  // Eigen 기하학적 변환 관련 기능 포함

// SE(3) 변환으로부터 변환 행렬과 쿼터니언을 얻는 함수
void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

// 벡터로부터 skew 대칭 행렬을 생성하는 함수
Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);

// EdgeAnalyticCostFunction 클래스 정의, ceres::SizedCostFunction 상속
class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
    public:
        // EdgeAnalyticCostFunction의 생성자
        EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_);
        virtual ~EdgeAnalyticCostFunction() {}  // 소멸자
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;  // 평가 함수

        Eigen::Vector3d curr_point;  // 현재 포인트
        Eigen::Vector3d last_point_a;  // 이전 포인트 A
        Eigen::Vector3d last_point_b;  // 이전 포인트 B
};

// SurfNormAnalyticCostFunction 클래스 정의, ceres::SizedCostFunction 상속
class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
    public:
        // SurfNormAnalyticCostFunction의 생성자
        SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
        virtual ~SurfNormAnalyticCostFunction() {}  // 소멸자
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;  // 평가 함수

        Eigen::Vector3d curr_point;  // 현재 포인트
        Eigen::Vector3d plane_unit_norm;  // 평면의 단위 법선 벡터
        double negative_OA_dot_norm;  // 음수 OA 벡터의 법선 내적
};

// 여기에서 업데이트
#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)
// Ceres 버전이 2.1 이상이거나 3 이상일 때, Manifold 클래스를 사용하여 PoseSE3Parameterization 정의
class PoseSE3Parameterization : public ceres::Manifold {
public:
    PoseSE3Parameterization() {}  // 생성자
    virtual ~PoseSE3Parameterization() {}  // 소멸자
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;  // Plus 함수
    virtual bool PlusJacobian(const double* x, double* jacobian) const;  // PlusJacobian 함수
    virtual bool Minus(const double* x, const double* delta, double* x_plus_delta) const;  // Minus 함수
    virtual bool MinusJacobian(const double* x, double* jacobian) const;  // MinusJacobian 함수
    virtual int AmbientSize() const { return 7; }  // 매개 공간의 크기 반환
    virtual int TangentSize() const { return 6; }  // 접선 공간의 크기 반환
};
#else
// Ceres 버전이 2.1 미만일 때, LocalParameterization 클래스를 사용하여 PoseSE3Parameterization 정의
class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization() {}  // 생성자
    virtual ~PoseSE3Parameterization() {}  // 소멸자
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;  // Plus 함수
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;  // Jacobian 계산 함수
    virtual int GlobalSize() const { return 7; }  // 전역 공간의 크기 반환
    virtual int LocalSize() const { return 6; }  // 로컬 공간의 크기 반환
};
#endif

#endif // _LIDAR_OPTIMIZATION_ANALYTIC_H_  // _LIDAR_OPTIMIZATION_ANALYTIC_H_ 정의 끝
