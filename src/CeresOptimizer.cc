/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <Eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


#include "Converter.h"
#include "CeresCostFuncs.h"

#include "CeresOptimizer.h"

#include<mutex>

#include<ceres/ceres.h>

#include <ceres/rotation.h>


namespace ORB_SLAM2 {


    void
    CeresOptimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                           const bool bRobust) {
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }


    void CeresOptimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                          int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                          const bool bRobust) {
    }

    int CeresOptimizer::PoseOptimization(Frame *pFrame) {
#if 1

        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < pFrame->N; i++) {
            pFrame->mvbOutlier[i] = false;
        }


        Eigen::Quaterniond q;
        Eigen::Vector3d t;

        cv::Mat rot = pFrame->mTcw.colRange(0, 3).rowRange(0, 3);
        cv::Mat trans = pFrame->mTcw.rowRange(0, 3).col(3);


        int inliers = 0;


        // 4 epoch optimize
        for (int epoch = 0; epoch < 4; epoch++) {
            ceres::Problem problem;

            Converter::toQuaternion(rot, q);
            cv::cv2eigen(trans, t);
//
//            std::cout << "------before optimize----------------" << std::endl;
//            std::cout << t << std::endl;

            ceres::LocalParameterization *qlp = new ceres::EigenQuaternionParameterization;


            for (int i = 0; i < pFrame->N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (!pFrame->mvbOutlier[i] && pMP) {

                    Eigen::Vector3d p3d;
                    Eigen::Vector2d p2d;
                    cv::cv2eigen(pMP->GetWorldPos(), p3d);

                    cv::KeyPoint &kp = pFrame->mvKeysUn[i];
                    //p2d(0) = (kp.pt.x - pFrame->cx) * pFrame->invfx;
                    //p2d(1) = (kp.pt.y - pFrame->cy) * pFrame->invfy;

                    p2d(0) = kp.pt.x;
                    p2d(1) = kp.pt.y;


                    ceres::LossFunction *loss_function = new ceres::HuberLoss(sqrt(5.991));

                    ceres::CostFunction *cost_function = ReprojectionOnlyPoseError::Create(p3d, p2d);

                    problem.AddResidualBlock(cost_function, loss_function, q.coeffs().data(), t.data());

                    problem.SetParameterization(q.coeffs().data(), qlp);

                }
            }


            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
//
//            std::cout << "---------after-------------" << std::endl;
//            std::cout << t << std::endl;
//
//            std::cout << summary.FullReport() << '\n';

            for (int i = 0; i < pFrame->N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (!pMP) {
                    continue;
                }


                Eigen::Vector3d p3d;
                Eigen::Vector2d p2d;
                cv::cv2eigen(pMP->GetWorldPos(), p3d);

                cv::KeyPoint &kp = pFrame->mvKeysUn[i];

                p2d(0) = kp.pt.x;
                p2d(1) = kp.pt.y;

                ReprojectionOnlyPoseError err(p3d, p2d);
                double residual[2];
                err(q.coeffs().data(), t.data(), residual);


                float invSigma2 = pFrame->mvInvLevelSigma2[kp.octave];
                Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

                Eigen::Vector2d vec(residual[0], residual[0]);
                float chi2 = vec.dot(information * vec);

                pFrame->mvbOutlier[i] = chi2 > 5.991;

                inliers += pFrame->mvbOutlier[i] ? 0 : 1;
            }
        }

        cv::Mat Rotation, Translate;
        Converter::toCvMat(q, Rotation);
        cv::eigen2cv(t, Translate);

        cv::Mat pose = cv::Mat::eye(4, 4, pFrame->mTcw.type());

        Rotation.copyTo(pose.rowRange(0, 3).colRange(0, 3));
        Translate.copyTo(pose.rowRange(0, 3).col(3));

        pFrame->SetPose(pose);

        return inliers;
#endif

    }


    void CeresOptimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap) {
    }


    void CeresOptimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                                const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                                const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                                const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                                const bool &bFixScale) {
    }

    int
    CeresOptimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1,
                                 g2o::Sim3 &g2oS12,
                                 const float th2, const bool bFixScale) {
        return 0;
    }


} //namespace ORB_SLAM
