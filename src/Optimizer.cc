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

#include "Optimizer.h"

#define USE_CERES

#ifdef USE_CERES
#include "CeresOptimizer.h"
#else
#include "G2Optimizer.h"
#endif

namespace ORB_SLAM2
{


void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
#ifdef USE_CERES
    CeresOptimizer::GlobalBundleAdjustemnt(pMap, nIterations, pbStopFlag, nLoopKF,bRobust);
#else
    G2Optimizer::GlobalBundleAdjustemnt(pMap, nIterations, pbStopFlag, nLoopKF,bRobust);
#endif
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
#ifdef USE_CERES
    CeresOptimizer::BundleAdjustment(vpKFs,vpMP,nIterations, pbStopFlag, nLoopKF, bRobust);
#else
    G2Optimizer::BundleAdjustment(vpKFs,vpMP,nIterations, pbStopFlag, nLoopKF, bRobust);
#endif
}

int Optimizer::PoseOptimization(Frame *pFrame)
{
#ifdef USE_CERES
    return CeresOptimizer::PoseOptimization(pFrame);
#else
    return G2Optimizer::PoseOptimization(pFrame);
#endif
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{
#ifdef USE_CERES
    CeresOptimizer::LocalBundleAdjustment(pKF,pbStopFlag,pMap);
#else
    G2Optimizer::LocalBundleAdjustment(pKF,pbStopFlag,pMap);
#endif
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
//#ifdef USE_CERES
//    CeresOptimizer::OptimizeEssentialGraph(pMap,pLoopKF, pCurKF,
//                                       NonCorrectedSim3,
//                                       CorrectedSim3,
//                                       LoopConnections, bFixScale);
//#else
//    G2Optimizer::OptimizeEssentialGraph(pMap,pLoopKF, pCurKF,
//                                           NonCorrectedSim3,
//                                           CorrectedSim3,
//                                           LoopConnections, bFixScale);
//#endif
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
#ifdef USE_CERES
    //CeresOptimizer::OptimizeSim3(pKF1, pKF2, vpMatches1, g2oS12, th2, bFixScale);
#else
    //G2Optimizer::OptimizeSim3(pKF1, pKF2, vpMatches1, g2oS12, th2, bFixScale);
#endif
    return 0;
}


} //namespace ORB_SLAM
