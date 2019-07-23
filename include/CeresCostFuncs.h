//
// Created by lyfpcy on 2019-07-06.
//

#ifndef ORB_SLAM2_CERESCOSTFUNCS_H
#define ORB_SLAM2_CERESCOSTFUNCS_H

#include<ceres/ceres.h>
#include <ceres/rotation.h>
#include <Sophus/se3.hpp>
#include <Sophus/sim3.hpp>

namespace ORB_SLAM2 {

    using namespace Sophus;

    const double fx = 517.306408;
    const double fy = 516.469215;
    const double cx = 318.643040;
    const double cy = 255.313989;


    class ReprojectionError {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionError(const Eigen::Vector2d &observed_p) : m_observed_p(observed_p) {}

        template<typename T>
        bool operator()(const T *const q, const T *const t,
                        const T *const point, T *residuals) const {

            Eigen::Map<const Eigen::Quaternion<T>> quat(q);
            Eigen::Map<const Eigen::Matrix<T,3,1>> P(point);

            Eigen::Matrix<T, 3, 1> predicted_p = quat * P;

            predicted_p[0] += t[0];
            predicted_p[1] += t[1];
            predicted_p[2] += t[2];

            predicted_p[0] /= predicted_p[2];
            predicted_p[1] /= predicted_p[2];

            // compute residuals
            residuals[0] = predicted_p(0)*T(fx)+T(cx) - T(m_observed_p(0));
            residuals[1] = predicted_p(1)*T(fy)+T(cy) - T(m_observed_p(1));

            return true;
        }

        static ceres::CostFunction *Create(Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
                    new ReprojectionError(pt2d)));
        }

    private:

        // observed 2D point
        Eigen::Vector2d m_observed_p;
    };


    class ReprojectionOnlyPoseError {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionOnlyPoseError(const Eigen::Vector3d &point3d, const Eigen::Vector2d &observed_p)
        : m_point3d(point3d), m_observed_p(observed_p) {}

        template<typename T>
        bool operator()(const T *const q, const T *const t, T *residuals) const {

            // project 3D object point to the image plane

            Eigen::Map<const Eigen::Quaternion<T> > quat(q);

            Eigen::Matrix<T, 3, 1> predicted_p = quat * m_point3d.template cast<T>();

            predicted_p[0] += t[0];
            predicted_p[1] += t[1];
            predicted_p[2] += t[2];

            predicted_p[0] /= predicted_p[2];
            predicted_p[1] /= predicted_p[2];

            // compute residuals
            residuals[0] = predicted_p(0)*T(fx)+T(cx) - T(m_observed_p(0));
            residuals[1] = predicted_p(1)*T(fy)+T(cy) - T(m_observed_p(1));

            return true;
        }

        static ceres::CostFunction *Create(Eigen::Vector3d &pt3d, Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<ReprojectionOnlyPoseError, 2, 4, 3>(
                    new ReprojectionOnlyPoseError(pt3d, pt2d)));
        }

    private:

        Eigen::Vector3d m_point3d;
        // observed 2D point
        Eigen::Vector2d m_observed_p;
    };


    class ReprojectionSim3Error {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionSim3Error(const Eigen::Vector2d &observed_p) : m_observed_p(observed_p) {}

        template<typename T>
        bool operator()(const T *const s, const T *const q, const T *const t,
                        const T *const point, T *residuals) const {

            Eigen::Map<const Eigen::Quaternion<T>> quat(q);
            Eigen::Map<const Eigen::Matrix<T,3,1>> P(point);

            Eigen::Matrix<T, 3, 1> predicted_p = quat * P;

            predicted_p[0] += t[0];
            predicted_p[1] += t[1];
            predicted_p[2] += t[2];

            predicted_p[0] /= predicted_p[2];
            predicted_p[1] /= predicted_p[2];

            // compute residuals
            residuals[0] = predicted_p(0)*T(fx)+T(cx) - T(m_observed_p(0));
            residuals[1] = predicted_p(1)*T(fy)+T(cy) - T(m_observed_p(1));

            return true;
        }

        static ceres::CostFunction *Create(Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<ReprojectionSim3Error, 2, 1, 4, 3, 3>(
                    new ReprojectionSim3Error(pt2d)));
        }

    private:

        // observed 2D point
        Eigen::Vector2d m_observed_p;
    };

    class PoseLieCostFunction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseLieCostFunction(const Eigen::Vector2d& observed, const Eigen::Vector3d& point)
        : observed_(observed),point_(point) {}

        template <typename T>
        bool operator()(const T* const pose_raw, T* residuals) const {

            Eigen::Map<Sophus::SE3<T> const> const pose(pose_raw);

            Eigen::Matrix<T, 3, 1> P = pose * point_.template cast<T>();
            P /= P[2];

            residuals[0] = P[0]*T(fx)+T(cx) - T(observed_[0]);
            residuals[1] = P[1]*T(fy)+T(cy) - T(observed_[1]);

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d &pt3d, const Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<PoseLieCostFunction, 2, 7>(
                    new PoseLieCostFunction(pt2d, pt3d)));
        }

    private:
        const Eigen::Vector2d observed_;
        const Eigen::Vector3d point_;
    };

    class ReprojLieCostFunction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojLieCostFunction(const Eigen::Vector2d& observed) : observed_(observed){}

        template <typename T>
        bool operator()(const T* const pose_raw, const T* const point_raw, T* residuals) const {

            Eigen::Map<Sophus::SE3<T> const> const pose(pose_raw);
            Eigen::Map<Eigen::Matrix<T, 3, 1> const> const point(point_raw);

            Eigen::Matrix<T, 3, 1> P = pose * point;
            P /= P[3];

            residuals[0] = P[0]*T(fx)+T(cx) - T(observed_[0]);
            residuals[1] = P[1]*T(fy)+T(cy) - T(observed_[1]);

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<ReprojLieCostFunction, 2, 7, 3>(
                    new ReprojLieCostFunction(pt2d)));
        }

    private:
        const Eigen::Vector2d observed_;
    };

    class LocalParameterizeSE3 : public ceres::LocalParameterization {
    public:
        virtual ~LocalParameterizeSE3() {}

        // SE3 plus operation for Ceres
        //
        //  T * exp(x)
        //
        virtual bool Plus(double const* T_raw, double const* delta_raw,
                          double* T_plus_delta_raw) const {
            Eigen::Map<SE3d const> const T(T_raw);
            Eigen::Map<Vector6d const> const delta(delta_raw);
            Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * SE3d::exp(delta);
            return true;
        }

        // Jacobian of SE3 plus operation for Ceres
        //
        // dx T * exp(x)  with  x=0
        //
        virtual bool ComputeJacobian(double const* T_raw,
                                     double* jacobian_raw) const {
            Eigen::Map<SE3d const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, 6, 7> , Eigen::RowMajor> jacobian(jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0().transpose();
            return true;
        }

        virtual int GlobalSize() const { return SE3d::num_parameters; }

        virtual int LocalSize() const { return SE3d::DoF; }
    };


    struct Sim3AdderFunctor {
        template<typename T>
        bool operator()(const T *T_raw, const T *delta_raw, T *x_plus_delta) const {
            Eigen::Map<Sim3<T> const> const raw(T_raw);
            Eigen::Map<Eigen::Matrix<T,7,1> const> const delta(delta_raw);
            Eigen::Map<Sim3<T>> T_plus_delta(x_plus_delta);
            T_plus_delta = raw * Sim3<T>::exp(delta);
            return true;
        }
    };


    class Sim3CostFunction
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Sim3CostFunction(const Sim3d &t_ab_measured)
                : t_ab_measured_(t_ab_measured) {}

        template <typename T>
        bool operator()(const T *const a_ptr, const T *const b_ptr,
                        T *residuals_ptr) const
        {

            Eigen::Map<Sophus::Sim3<T> const> const t_a(a_ptr);
            Eigen::Map<Sophus::Sim3<T> const> const t_b(b_ptr);

            Eigen::Map<Eigen::Matrix<T, 7, 1>> residuals(residuals_ptr);

            residuals = (t_ab_measured_.template cast<T>().inverse() * t_a.inverse() * t_b).log();

            return true;
        }

        static ceres::CostFunction* createCost(Sim3d Sji) {
            Sim3CostFunction *functor = new Sim3CostFunction(Sji);

            ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<Sim3CostFunction,7,7,7>(functor);
            return cost;
        }

    private:
        // The measurement for the position of B relative to A in the A frame.
        const Sim3d t_ab_measured_;
        // The square root of the measurement information matrix.
        //const Eigen::Matrix<double, 7, 7> sqrt_information_;
    };

    class ReprojSim3CostFunction {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojSim3CostFunction(const Eigen::Vector2d& observed, const Eigen::Vector3d& point, bool inverse, const Eigen::Matrix2d& info)
        : observed_(observed), point_(point), inverse_(inverse), information_(info) {}

        template <typename T>
        bool operator()(const T* const pose_raw, T* residuals) const {

            Eigen::Map<Sophus::Sim3<T> const> const pose(pose_raw);

            Eigen::Matrix<T, 3, 1> P = inverse_ ? pose.inverse() * point_.template cast<T>() : pose * point_.template cast<T>();
            P /= P[3];

            residuals[0] = P[0]*T(fx)+T(cx) - T(observed_[0]);
            residuals[1] = P[1]*T(fy)+T(cy) - T(observed_[1]);

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector2d &pt2d, Eigen::Vector3d& point, bool inverse, const Eigen::Matrix2d& info) {
            return (new ceres::AutoDiffCostFunction<ReprojSim3CostFunction, 2, 7>(
                    new ReprojSim3CostFunction(pt2d, point, inverse, info)));
        }


        double chi2(const Sim3d& sim3) {
            Eigen::Vector2d err;
            operator()(sim3.data(), err.data());

            return err.dot(information_*err);
        }

    private:
        const Eigen::Vector2d observed_;
        const Eigen::Vector3d point_;
        const bool inverse_;
        const Eigen::Matrix2d information_;
    };

}

#endif //ORB_SLAM2_CERESCOSTFUNCS_H
