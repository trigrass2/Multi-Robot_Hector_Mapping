//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
// Modified by Muhammet Balcilar. Yildiz Technical University, Istanbul, 2016
// Any question please contact muhammetbalcilar@gmail.com

#ifndef _scanmatcher_h__
#define _scanmatcher_h__

#include <Eigen/Geometry>
#include "../scan/DataPointContainer.h"
#include "../util/UtilFunctions.h"

#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"

namespace hectorslam{

template<typename ConcreteOccGridMapUtil>
class ScanMatcher
{
public:

  ScanMatcher(DrawInterface* drawInterfaceIn = 0, HectorDebugInfoInterface* debugInterfaceIn = 0)
    : drawInterface(drawInterfaceIn)
    , debugInterface(debugInterfaceIn)
  {}

  ~ScanMatcher()
  {}

  Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, ConcreteOccGridMapUtil& gridMapUtil, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix, int maxIterations,int rid)
  {

    if (dataContainer.getSize() != 0) {

      Eigen::Vector3f beginEstimateMap(gridMapUtil.getMapCoordsPose(beginEstimateWorld));
      Eigen::Vector3f estimate(beginEstimateMap);
      estimateTransformationLogLh(estimate, gridMapUtil, dataContainer,rid);

      //bool notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

      /*
      const Eigen::Matrix2f& hessian (H.block<2,2>(0,0));
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(hessian);
      const Eigen::Vector2f& eigValues (eig.eigenvalues());
      float cond = eigValues[1] / eigValues[0];
      float determinant = (hessian.determinant());
      */
      //std::cout << "\n cond: " << cond << " det: " << determinant << "\n";

      Eigen::Matrix3f H = Eigen::Matrix3f::Zero();


      int numIter = maxIterations;
      for (int i = 0; i < numIter; ++i)
      {

        estimateTransformationLogLh(estimate, gridMapUtil, dataContainer,rid);
        //notConverged = estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

      }

      estimate[2] = util::normalize_angle(estimate[2]);
      covMatrix = Eigen::Matrix3f::Zero();     
      covMatrix = H;
      return gridMapUtil.getWorldCoordsPose(estimate);
    }

    return beginEstimateWorld;
  }

protected:

  bool estimateTransformationLogLh(Eigen::Vector3f& estimate, ConcreteOccGridMapUtil& gridMapUtil, const DataContainer& dataPoints,int rid)
  {
      Eigen::Vector3f dTr;
      Eigen::Matrix3f H;
    gridMapUtil.getCompleteHessianDerivs(estimate, dataPoints, H, dTr,rid);
    //std::cout << "\nH\n" << H  << "\n";
    //std::cout << "\ndTr\n" << dTr  << "\n";
    if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f))
    {
      //H += Eigen::Matrix3f::Identity() * 1.0f;
      //  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H);
      //  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);





      Eigen::Vector3f searchDir (H.inverse() * dTr);      



      if (searchDir[2] > 0.2f) {
        searchDir[2] = 0.2f;
        std::cout << "SearchDir angle change too large\n";
      } else if (searchDir[2] < -0.2f) {
        searchDir[2] = -0.2f;
        std::cout << "SearchDir angle change too large\n";
      }
      updateEstimatedPose(estimate, searchDir);
      return true;
    }
    return false;
  }

  void updateEstimatedPose(Eigen::Vector3f& estimate, const Eigen::Vector3f& change)
  {
    estimate += change;
  }

  void drawScan(const Eigen::Vector3f& pose, const ConcreteOccGridMapUtil& gridMapUtil, const DataContainer& dataContainer)
  {
    drawInterface->setScale(0.02);

    Eigen::Affine2f transform(gridMapUtil.getTransformForState(pose));

    int size = dataContainer.getSize();
    for (int i = 0; i < size; ++i) {
      const Eigen::Vector2f& currPoint (dataContainer.getVecEntry(i));
      drawInterface->drawPoint(gridMapUtil.getWorldCoordsPoint(transform * currPoint));
    }
  }

protected:


  DrawInterface* drawInterface;
  HectorDebugInfoInterface* debugInterface;
};

}


#endif
