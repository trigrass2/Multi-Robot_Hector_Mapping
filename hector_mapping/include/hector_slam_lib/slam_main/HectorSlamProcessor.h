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

#ifndef _hectorslamprocessor_h__
#define _hectorslamprocessor_h__

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"
#include "../scan/DataPointContainer.h"

#include "../util/UtilFunctions.h"
#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"
#include "../util/MapLockerInterface.h"

#include "MapRepresentationInterface.h"
#include "MapRepMultiMap.h"


#include <float.h>

namespace hectorslam{

class HectorSlamProcessor
{
public:    

  HectorSlamProcessor(float mapResolution, int mapSizeX, int mapSizeY , const Eigen::Vector2f& startCoords, int multi_res_size, DrawInterface* drawInterfaceIn = 0, HectorDebugInfoInterface* debugInterfaceIn = 0)
    : drawInterface(drawInterfaceIn)
    , debugInterface(debugInterfaceIn)
  {
    mapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords, drawInterfaceIn, debugInterfaceIn);

    this->reset();

    this->setMapUpdateMinDistDiff(0.4f *1.0f);
    this->setMapUpdateMinAngleDiff(0.13f * 1.0f);
  }

  ~HectorSlamProcessor()
  {
    delete mapRep;
  }

  void update(const DataContainer& dataContainer, const Eigen::Vector3f& poseHintWorld, int rid,bool map_without_matching = false)
  {
    //std::cout << "\nph:\n" << poseHintWorld << "\n";

    Eigen::Vector3f newPoseEstimateWorld;
    Eigen::Matrix3f lastcov;

    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            lastcov(i,j)=lastScanMatchCov(i,rid*3+j);


    if (!map_without_matching){
        newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastcov,rid));
    }else{
        newPoseEstimateWorld = poseHintWorld;
    }


    lastScanMatchPose(0,rid) = newPoseEstimateWorld[0];
    lastScanMatchPose(1,rid) = newPoseEstimateWorld[1];
    lastScanMatchPose(2,rid) = newPoseEstimateWorld[2];

    Eigen::Vector3f lastMapUpdatePoseV;
    lastMapUpdatePoseV[0]=lastMapUpdatePose(0,rid);
    lastMapUpdatePoseV[1]=lastMapUpdatePose(1,rid);
    lastMapUpdatePoseV[2]=lastMapUpdatePose(2,rid);


    if(util::poseDifferenceLargerThan(newPoseEstimateWorld, lastMapUpdatePoseV, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || map_without_matching)
    {
      registermutex.lock();

      mapRep->updateByScan(dataContainer, newPoseEstimateWorld,rid);
      mapRep->onMapUpdated();

      registermutex.unlock();


      lastMapUpdatePose(0,rid) = newPoseEstimateWorld[0];
      lastMapUpdatePose(1,rid) = newPoseEstimateWorld[1];
      lastMapUpdatePose(2,rid) = newPoseEstimateWorld[2];
    }



    if(drawInterface){
      const GridMap& gridMapRef (mapRep->getGridMap());
      drawInterface->setColor(1.0, 0.0, 0.0);
      drawInterface->setScale(0.15);

      drawInterface->drawPoint(gridMapRef.getWorldCoords(Eigen::Vector2f::Zero()));
      drawInterface->drawPoint(gridMapRef.getWorldCoords((gridMapRef.getMapDimensions().array()-1).cast<float>()));
      drawInterface->drawPoint(Eigen::Vector2f(1.0f, 1.0f));

      drawInterface->sendAndResetData();
    }

    if (debugInterface)
    {
      debugInterface->sendAndResetData();
    }
  }

  void update3(const DataContainer& dataContainer, const Eigen::Vector3f& poseHintWorld, int rid)
  {
      mapRep->updateByScan(dataContainer, poseHintWorld,rid);
      mapRep->onMapUpdated();

  }

  void update2(const DataContainer& dataContainer, const Eigen::Vector3f& poseHintWorld, int rid,bool map_without_matching = false)
  {
    //std::cout << "\nph:\n" << poseHintWorld << "\n";

    Eigen::Vector3f newPoseEstimateWorld;
    Eigen::Matrix3f lastcov;

    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            lastcov(i,j)=lastScanMatchCov(i,rid*3+j);


    if (!map_without_matching){
        newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastcov,rid));
    }else{
        newPoseEstimateWorld = poseHintWorld;
    }
    lastScanMatchPose(0,rid) = newPoseEstimateWorld[0];
    lastScanMatchPose(1,rid) = newPoseEstimateWorld[1];
    lastScanMatchPose(2,rid) = newPoseEstimateWorld[2];
  }



  void reset()
  {
    //lastMapUpdatePose = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
    lastMapUpdatePose = Eigen::Matrix<float, 3, 8>::Zero();
    lastScanMatchPose = Eigen::Matrix<float, 3, 8>::Zero();
    mapRep->reset();
  }

  const Eigen::Matrix<float, 3, 8>& getLastScanMatchPose() const
  {
//      Eigen::Vector3f loc;
//      loc[0]=lastScanMatchPose(0,rid);
//      loc[1]=lastScanMatchPose(1,rid);
//      loc[2]=lastScanMatchPose(2,rid);
//      return loc; //
      return lastScanMatchPose;
  };



  const Eigen::Matrix<float, 3, 24>& getLastScanMatchCovariance() const
  {
//      Eigen::Matrix3f& cov;
//      for (int i=0;i<3;i++)
//          for (int j=0;j<3;j++)
//              cov(i,j)=lastScanMatchCov(i,rid*3+j);

//      return cov;//
      return lastScanMatchCov;
  };

  float getScaleToMap() const { return mapRep->getScaleToMap(); };

  int getMapLevels() const { return mapRep->getMapLevels(); };
  const GridMap& getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };

  void addMapMutex(int i, MapLockerInterface* mapMutex) { mapRep->addMapMutex(i, mapMutex); };
  MapLockerInterface* getMapMutex(int i) { return mapRep->getMapMutex(i); };

  void setUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
  void setUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
  void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
  void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

protected:

  MapRepresentationInterface* mapRep;

  Eigen::Matrix<float, 3, 8> lastMapUpdatePose;
  Eigen::Matrix<float, 3, 8> lastScanMatchPose;
  Eigen::Matrix<float, 3, 24> lastScanMatchCov;

  float paramMinDistanceDiffForMapUpdate;
  float paramMinAngleDiffForMapUpdate;

  DrawInterface* drawInterface;
  HectorDebugInfoInterface* debugInterface;

  boost::mutex registermutex;
};

}

#endif
