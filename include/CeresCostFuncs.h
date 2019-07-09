//
// Created by lyfpcy on 2019-07-06.
//

#ifndef ORB_SLAM2_CERESCOSTFUNCS_H
#define ORB_SLAM2_CERESCOSTFUNCS_H

#include<ceres/ceres.h>
#include <ceres/rotation.h>

namespace ORB_SLAM2 {

    /*
    class ReprojectionError {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionError(const Eigen::Vector2d &observed_p) : m_observed_p(observed_p) {}

        template<typename T>
        bool operator()(const T *const q, const T *const t,
                        const T *const point, T *residuals) const {
            Eigen::Matrix<T, 3, 1> P;
            P(0) = T(point[0]);
            P(1) = T(point[1]);
            P(2) = T(point[2]);


            // project 3D object point to the image plane
            Eigen::Matrix<T, 3, 1> predicted_p;

            // Convert quaternion from Eigen convention (x, y, z, w)
            // to Ceres convention (w, x, y, z)
            T q_ceres[4] = {q[3], q[0], q[1], q[2]};

            ceres::QuaternionRotatePoint(q_ceres, P, predicted_p);

            predicted_p[0] += t[0];
            predicted_p[1] += t[1];
            predicted_p[2] += t[2];

            predicted_p /= predicted_p[2];

            // compute residuals
            residuals[0] = predicted_p(0) - T(m_observed_p(0));
            residuals[1] = predicted_p(1) - T(m_observed_p(1));

            return true;
        }

        static ceres::CostFunction *Create(Eigen::Vector2d &pt2d) {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 10, 3>(
                    new ReprojectionError(pt2d)));
        }

    private:

        // observed 2D point
        Eigen::Vector2d m_observed_p;
    };
     */

    const double fx = 517.306408;
    const double fy = 516.469215;
    const double cx = 318.643040;
    const double cy = 255.313989;


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
            residuals[1] = predicted_p(1)*T(fx)+T(cx) - T(m_observed_p(1));

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
}

#endif //ORB_SLAM2_CERESCOSTFUNCS_H
