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
#include <Sophus/se3.hpp>
#include <Sophus/sim3.hpp>



namespace ORB_SLAM2 {

    using namespace Sophus;
    using VecSim3 = vector<Sim3d,Eigen::aligned_allocator<Sim3d> >;


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

        long unsigned int maxKFid = 0;

        ceres::Problem problem;
        ceres::LocalParameterization *qlp = new LocalParameterizeSE3();
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering;


        KeyFrame *startFrame = nullptr;
        // Set KeyFrame vertices
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            // remember max frameid
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;

            if (pKF->mnId == 0) {
                startFrame = pKF;
            }
        }

        //================================================================
        // construct ceres optimize redisual function

        for (size_t i = 0; i < vpMP.size(); i++) {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;

            pMP->mWPos = Converter::toVector3d(pMP->GetWorldPos());

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //SET EDGES
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                pKF->ToSE3();

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
                Eigen::Vector2d p2d(kpUn.pt.x, kpUn.pt.y);

                // calculate reproject error with huber loss
                ceres::LossFunction *loss_function = new ceres::HuberLoss(sqrt(5.991));

                ceres::CostFunction *cost_function = ReprojLieCostFunction::Create(p2d);

                problem.AddResidualBlock(cost_function, loss_function, pKF->pose_ba_, pMP->mWPos.data());

                problem.SetParameterization(pKF->pose_ba_, qlp);

                //set marginalize order
                ordering->AddElementToGroup(pMP->mWPos.data(), 0);
                ordering->AddElementToGroup(pKF->pose_ba_, 1);
            }
        }


        if (startFrame != nullptr) {
            problem.SetParameterBlockConstant(startFrame->pose_ba_);
        }


        //================================================================
        //begin optimize

        options.linear_solver_ordering.reset(ordering);

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.FullReport() << '\n';


        //================================================================
        // Recover optimized data

        //Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;

            if (nLoopKF == 0) {
                pKF->FromSE3();
            } else {
                Eigen::Map<Sophus::SE3d> se3(pKF->pose_ba_);
                Sophus::SE3d ss = se3;
                Converter::toCvMat(ss, pKF->mTcwGBA);
                pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        //Points
        for (size_t i = 0; i < vpMP.size(); i++) {

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;

            cv::Mat pos = Converter::toCvMat(pMP->mWPos);

            if (nLoopKF == 0) {
                pMP->SetWorldPos(pos);
                pMP->UpdateNormalAndDepth();
            } else {
                pMP->mPosGBA.create(3, 1, CV_32F);
                pos.copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }


    }

    int CeresOptimizer::PoseOptimization(Frame *pFrame) {
#if 0

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < pFrame->N; i++) {
            pFrame->mvbOutlier[i] = false;
        }


        Eigen::Quaterniond q;
        Eigen::Vector3d t;

        cv::Mat rot = pFrame->mTcw.colRange(0, 3).rowRange(0, 3);
        cv::Mat trans = pFrame->mTcw.rowRange(0, 3).col(3);


        int inliers = 0;
        int all = 0;


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


                    ceres::LossFunction *loss_function = epoch > 1 ? nullptr : new ceres::HuberLoss(sqrt(5.991));

                    ceres::CostFunction *cost_function = ReprojectionOnlyPoseError::Create(p3d, p2d);

                    problem.AddResidualBlock(cost_function, loss_function, q.coeffs().data(), t.data());

                    problem.SetParameterization(q.coeffs().data(), qlp);

                }
            }


            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            //std::cout << summary.FullReport() << '\n';

            inliers = 0;
            all = 0;
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
                all += 1;
            }
        }

        cv::Mat Rotation, Translate;
        Converter::toCvMat(q, Rotation);
        cv::eigen2cv(t, Translate);

        cv::Mat pose = cv::Mat::eye(4, 4, pFrame->mTcw.type());

        Rotation.copyTo(pose.rowRange(0, 3).colRange(0, 3));
        Translate.copyTo(pose.rowRange(0, 3).col(3));


//        std::cout << "---------before-------------" << std::endl;
//        std::cout << "all: " << all << ", inliers: " << inliers << std::endl;
//        std::cout << pFrame->mTcw << std::endl;

        pFrame->SetPose(pose);

//        std::cout << "---------after-------------" << std::endl;
//        std::cout << pose << std::endl;
//
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//
//        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//        std::cout << "---------cost: " << ttrack << std::endl;

        return inliers;
#else
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < pFrame->N; i++) {
            pFrame->mvbOutlier[i] = false;
        }


        Sophus::SE3d pose;


        int inliers = 0;
        int all = 0;


        // 4 epoch optimize
        for (int epoch = 0; epoch < 4; epoch++) {
            ceres::Problem problem;

            Converter::toSE3(pFrame->mTcw, pose);
//
//            std::cout << "------before optimize----------------" << std::endl;
//            std::cout << t << std::endl;

            ceres::LocalParameterization *qlp = new LocalParameterizeSE3();


            for (int i = 0; i < pFrame->N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (!pFrame->mvbOutlier[i] && pMP) {

                    Eigen::Vector3d p3d;
                    cv::cv2eigen(pMP->GetWorldPos(), p3d);

                    cv::KeyPoint &kp = pFrame->mvKeysUn[i];
                    Eigen::Vector2d p2d(kp.pt.x,kp.pt.y);



                    ceres::LossFunction *loss_function = epoch > 1 ? nullptr : new ceres::HuberLoss(sqrt(5.991));

                    ceres::CostFunction *cost_function = PoseLieCostFunction::Create(p3d, p2d);

                    problem.AddResidualBlock(cost_function, loss_function, pose.data());

                    problem.SetParameterization(pose.data(), qlp);

                }
            }


            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            //std::cout << summary.FullReport() << '\n';

            inliers = 0;
            all = 0;
            for (int i = 0; i < pFrame->N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (!pMP) {
                    continue;
                }


                Eigen::Vector3d p3d;
                cv::cv2eigen(pMP->GetWorldPos(), p3d);

                cv::KeyPoint &kp = pFrame->mvKeysUn[i];
                Eigen::Vector2d p2d(kp.pt.x,kp.pt.y);

                PoseLieCostFunction err(p2d, p3d);
                double residual[2];
                err(pose.data(), residual);


                float invSigma2 = pFrame->mvInvLevelSigma2[kp.octave];
                Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

                Eigen::Vector2d vec(residual[0], residual[0]);
                float chi2 = vec.dot(information * vec);

                pFrame->mvbOutlier[i] = chi2 > 5.991;

                inliers += pFrame->mvbOutlier[i] ? 0 : 1;
                all += 1;
            }
        }


        cv::Mat cvPose;
        Converter::toCvMat(pose, cvPose);


//        std::cout << "---------before-------------" << std::endl;
//        std::cout << "all: " << all << ", inliers: " << inliers << std::endl;
//        std::cout << pFrame->mTcw << std::endl;

        pFrame->SetPose(cvPose);

//        std::cout << "---------after-------------" << std::endl;
//        std::cout << pose << std::endl;
//
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//
//        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//        std::cout << "---------cost: " << ttrack << std::endl;

        return inliers;
#endif

    }


    void CeresOptimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap) {

#if 1

        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        lLocalKeyFrames.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;

        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        list<MapPoint *> lLocalMapPoints;

        for (auto &lkf : lLocalKeyFrames) {
            vector<MapPoint *> vpMPs = lkf->GetMapPointMatches();
            for (auto &mp : vpMPs) {
                if (!mp || mp->isBad()) {
                    continue;
                }

                if (mp->mnBALocalForKF != pKF->mnId) {
                    lLocalMapPoints.push_back(mp);
                    mp->mnBALocalForKF = pKF->mnId;
                }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        list<KeyFrame *> lFixedCameras;

        for (auto &mp : lLocalMapPoints) {
            map<KeyFrame *, size_t> observations = mp->GetObservations();
            for (auto &kfn : observations) {
                KeyFrame *pKFi = kfn.first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad())
                        lFixedCameras.push_back(pKFi);
                }
            }
        }

        // setup ceres optimizer

        ceres::Problem problem;
        ceres::LocalParameterization *qlp = new LocalParameterizeSE3;
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering;


        unsigned long maxKFid = 0;

        KeyFrame *startFrame = nullptr;
        // Set Local KeyFrame vertices
        for (auto &kf : lLocalKeyFrames) {

            if (kf->mnId == 0) {
                startFrame = kf;
            }

            if (kf->mnId > maxKFid)
                maxKFid = kf->mnId;
        }


        // Set Fixed KeyFrame vertices
        for (auto &kf : lFixedCameras) {
            if (kf->mnId > maxKFid)
                maxKFid = kf->mnId;
        }


        for (auto &mp : lLocalMapPoints) {
            const map<KeyFrame *, size_t> observations = mp->GetObservations();

            mp->mWPos = Converter::toVector3d(mp->GetWorldPos());

            for (auto &kfn : observations) {
                KeyFrame *pKF = kfn.first;
                size_t idx = kfn.second;

                if (pKF->isBad()) continue;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[idx];
                pKF->ToSE3();

                Eigen::Matrix<double, 2, 1> obs(kpUn.pt.x, kpUn.pt.y);

                // calculate reproject error with huber loss
                ceres::LossFunction *loss_function = new ceres::HuberLoss(sqrt(5.991));

                ceres::CostFunction *cost_function = ReprojLieCostFunction::Create(obs);

                problem.AddResidualBlock(cost_function, loss_function, pKF->pose_ba_, mp->mWPos.data());

                problem.SetParameterization(pKF->pose_ba_, qlp);

                //set marginalize order
                ordering->AddElementToGroup(mp->mWPos.data(), 0);
                ordering->AddElementToGroup(pKF->pose_ba_, 1);

            }
        }

        if (startFrame != nullptr) {
            problem.SetParameterBlockConstant(startFrame->pose_ba_);
        }


        // Set Fixed KeyFrame vertices
        for (auto &kf : lFixedCameras) {
            problem.SetParameterBlockConstant(kf->pose_ba_);
        }


        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        //================================================================
        //begin optimize

        options.linear_solver_ordering.reset(ordering);

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //------------------------------------------------
        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        if (bDoMore) {
//
//            // Check inlier observations
//            for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
//            {
//                g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
//                MapPoint* pMP = vpMapPointEdgeMono[i];
//
//                if(pMP->isBad())
//                    continue;
//
//                if(e->chi2()>5.991 || !e->isDepthPositive())
//                {
//                    e->setLevel(1);
//                }
//
//                e->setRobustKernel(0);
//            }
//
//            // Optimize again without the outliers
//
//            optimizer.initializeOptimization(0);
//            optimizer.optimize(10);

// TODO optimize again without outliers

        }
//
//        vector<pair<KeyFrame*,MapPoint*> > vToErase;
//        vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
//
//        // Check inlier observations
//        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
//        {
//            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
//            MapPoint* pMP = vpMapPointEdgeMono[i];
//
//            if(pMP->isBad())
//                continue;
//
//            if(e->chi2()>5.991 || !e->isDepthPositive())
//            {
//                KeyFrame* pKFi = vpEdgeKFMono[i];
//                vToErase.push_back(make_pair(pKFi,pMP));
//            }
//        }
//
//        // Get Map Mutex
//        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
//
//        if(!vToErase.empty())
//        {
//            for(size_t i=0;i<vToErase.size();i++)
//            {
//                KeyFrame* pKFi = vToErase[i].first;
//                MapPoint* pMPi = vToErase[i].second;
//                pKFi->EraseMapPointMatch(pMPi);
//                pMPi->EraseObservation(pKFi);
//            }
//        }
// TODO at the end, calculate chi2 error, erase outlier

        // Recover optimized data

        //Keyframes
        for (auto &kf : lLocalKeyFrames) {
            kf->FromSE3();
        }

        //Points
        for (auto &mp : lLocalMapPoints) {
            cv::Mat pos = Converter::toCvMat(mp->mWPos);

            mp->SetWorldPos(pos);
            mp->UpdateNormalAndDepth();
        }

#endif

    }


    void addPosegraphResidual(VecSim3& vScw, const VecSim3& vNCScw, ceres::Problem& problem, int i, int j) {
        const Sim3d Sji = vNCScw[j] * vNCScw[i].inverse();
        problem.AddResidualBlock(Sim3CostFunction::createCost(Sji), nullptr, vScw[i].data(), vScw[j].data());
    }

    void CeresOptimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                                const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                                const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                                const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                                const bool &bFixScale) {
#if 1

        const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        VecSim3 vScw(nMaxKFid+1), vNCScw(nMaxKFid+1);

        VecSim3 vCorrectedSwc(nMaxKFid+1);

        ceres::Problem problem;
        ceres::LocalParameterization *local_parameterization = new ceres::AutoDiffLocalParameterization<Sim3AdderFunctor,7,7>();


        const int minFeat = 100;

        // Set KeyFrame vertices
        for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
        {
            KeyFrame* pKF = vpKFs[i];
            if(pKF->isBad())
                continue;

            const int nIDi = pKF->mnId;


            auto it = CorrectedSim3.find(pKF);

            if(it!=CorrectedSim3.end())
            {
                //vScw[nIDi] = it->second;
                Converter::toSim3(it->second, vScw[nIDi]);
                //VSim3->setEstimate(it->second);
            }
            else
            {
                Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
                Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
                //g2o::Sim3 Siw(Rcw,tcw,1.0);
                Sophus::Sim3d Siw(Eigen::Quaterniond(Rcw), tcw);
                Siw.setScale(1.0);

                vScw[nIDi] = Siw;
                //VSim3->setEstimate(Siw);
            }


            auto nit = NonCorrectedSim3.find(pKF);
            if (nit != NonCorrectedSim3.end()) {
                Sim3d nsim;
                Converter::toSim3(nit->second, nsim);
                vNCScw[nIDi] = nsim;
            } else {
                vNCScw[nIDi] = vScw[nIDi];
            }


            problem.AddParameterBlock(vScw[nIDi].data(), 7, local_parameterization);

            if (nIDi == pLoopKF->mnId)
                problem.SetParameterBlockConstant(vScw[nIDi].data());

        }


        set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

        //const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

        // Set Loop edges
        for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame*> &spConnections = mit->second;

            for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                    continue;

                addPosegraphResidual(vScw, vNCScw, problem, nIDi, nIDj);

                sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
            }
        }

        // Set normal edges
        for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
        {
            KeyFrame* pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            Sim3d Swi;

            auto iti = NonCorrectedSim3.find(pKF);

            if(iti!=NonCorrectedSim3.end())
                //Swi = (iti->second).inverse();
                Converter::toSim3((iti->second).inverse(), Swi);
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame* pParentKF = pKF->GetParent();

            // Spanning tree edge
            if(pParentKF)
            {
                int nIDj = pParentKF->mnId;
                addPosegraphResidual(vScw, vNCScw, problem, nIDi, nIDj);
            }

            // Loop edges
            const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
            for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
            {
                KeyFrame* pLKF = *sit;
                if(pLKF->mnId<pKF->mnId)
                {
                    int nIDj = pLKF->mnId;

                    addPosegraphResidual(vScw, vNCScw, problem, nIDi, nIDj);
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
            {
                KeyFrame* pKFn = *vit;
                if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
                {
                    if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                    {
                        if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                            continue;

                        int nIDj = pKFn->mnId;
                        addPosegraphResidual(vScw, vNCScw, problem, nIDi, nIDj);
                    }
                }
            }
        }

        // Optimize!
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);



        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for(size_t i=0;i<vpKFs.size();i++)
        {
            KeyFrame* pKFi = vpKFs[i];

            const int nIDi = pKFi->mnId;

            Sim3d CorrectedSiw =  vScw[nIDi].inverse();
            Eigen::Matrix3d eigR = CorrectedSiw.rotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            int nIDr;
            if(pMP->mnCorrectedByKF==pCurKF->mnId)
            {
                nIDr = pMP->mnCorrectedReference;
            }
            else
            {
                KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }


            Sim3d Srw = vScw[nIDr];
            Sim3d correctedSwr = vCorrectedSwc[nIDr];

            cv::Mat P3Dw = pMP->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr * Srw * eigP3Dw;

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMP->SetWorldPos(cvCorrectedP3Dw);

            pMP->UpdateNormalAndDepth();
        }
#endif

    }

    int
    CeresOptimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1,
                                 g2o::Sim3 &g2oS12,
                                 const float th2, const bool bFixScale) {
#if 1
        ceres::Problem problem;
        ceres::LocalParameterization *localp = new ceres::AutoDiffLocalParameterization<Sim3AdderFunctor,7,7>();

        // Camera poses
        const cv::Mat R1w = pKF1->GetRotation();
        const cv::Mat t1w = pKF1->GetTranslation();
        const cv::Mat R2w = pKF2->GetRotation();
        const cv::Mat t2w = pKF2->GetTranslation();

        Sim3d s12;
        Converter::toSim3(g2oS12, s12);
        problem.AddParameterBlock(s12.data(), 7, localp);


        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2*N);

        vector<pair<ceres::ResidualBlockId,ReprojSim3CostFunction*>> residuals12(2*N);
        vector<pair<ceres::ResidualBlockId,ReprojSim3CostFunction*>> residuals21(2*N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for(int i=0; i<N; i++)
        {
            if(!vpMatches1[i])
                continue;

            MapPoint* pMP1 = vpMapPoints1[i];
            MapPoint* pMP2 = vpMatches1[i];


            if (!pMP1 || !pMP2)
                continue;

            const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(pMP1->isBad() || pMP2->isBad() || i2 < 0)
                continue;

            nCorrespondences++;


            cv::Mat P3D1c = R1w*pMP1->GetWorldPos()+t1w;
            cv::Mat P3D2c = R2w*pMP2->GetWorldPos()+t2w;

            Eigen::Vector3d p3d1, p3d2;
            cv::cv2eigen(P3D1c, p3d1);
            cv::cv2eigen(P3D2c, p3d2);

            // Set edge x1 = S12*X2

            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            Eigen::Matrix<double,2,1> obs1(kpUn1.pt.x, kpUn1.pt.y);

            float invSigma = pKF2->mvInvLevelSigma2[kpUn1.octave];
            Eigen::Matrix2d information = Eigen::Matrix2d::Identity()*invSigma;
            ReprojSim3CostFunction* functor = new ReprojSim3CostFunction(obs1, p3d2, false, information);
            ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<ReprojSim3CostFunction, 2, 7>(functor);
            ceres::ResidualBlockId rid = problem.AddResidualBlock(cost, new ceres::HuberLoss(deltaHuber), s12.data());

            residuals21.push_back(make_pair(rid, functor));


            // Set edge x2 = S21*X1

            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            Eigen::Matrix<double,2,1> obs2(kpUn2.pt.x, kpUn2.pt.y);

            float invSigma2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            Eigen::Matrix2d information2 = Eigen::Matrix2d::Identity()*invSigma2;
            ReprojSim3CostFunction* functor2 = new ReprojSim3CostFunction(obs2, p3d1, true, information2);
            ceres::CostFunction* cost2 = new ceres::AutoDiffCostFunction<ReprojSim3CostFunction, 2, 7>(functor2);
            ceres::ResidualBlockId rid2 = problem.AddResidualBlock(cost2, new ceres::HuberLoss(deltaHuber), s12.data());

            residuals12.push_back(make_pair(rid2, functor2));

            vnIndexEdge.push_back(i);

        }

        // Optimize!
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Check inliers
        int nBad=0;
        for(size_t i=0; i<residuals12.size();i++)
        {
            auto e12 = residuals12[i];
            auto e21 = residuals21[i];

            if(e12.second->chi2(s12)>th2 || e21.second->chi2(s12)>th2)
            {
                problem.RemoveResidualBlock(e12.first);
                problem.RemoveResidualBlock(e21.first);

                e21.second = nullptr;
                e12.second = nullptr;

                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = nullptr;
                nBad++;
            }
        }

        std::cout << "outlier: " << nCorrespondences-nBad << std::endl;


        if(nCorrespondences-nBad<10)
            return 0;

        // Optimize again only with inliers
        Converter::toSim3(g2oS12, s12);
        ceres::Solve(options, &problem, &summary);

        int nIn = 0;
        for(size_t i=0; i<residuals12.size();i++)
        {
            auto e12 = residuals12[i];
            auto e21 = residuals21[i];

            if (e12.second == nullptr || e21.second == nullptr)
                continue;

            if(e12.second->chi2(s12)>th2 || e21.second->chi2(s12)>th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = nullptr;
            }
            else
                nIn++;
        }

        // Recover optimized Sim3
        g2oS12 = g2o::Sim3(s12.rotationMatrix(), s12.translation(), s12.scale());

        std::cout << "outlier: " << nBad << ", inlier:" << nIn << std::endl;

        return nIn;
#else
        return 0;
#endif
    }


} //namespace ORB_SLAM
