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

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(false)
{
    bekleme=0;
    okuindex=0;
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);


  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

   private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
   private_nh_.param("map_frame", p_map_frame_, std::string("map"));

   std::string baseframe,odomframe,scantopic,scanmatchframe;
   scanoku0=0;scanyaz0=0;
   scanoku1=0;scanyaz1=0;
   scanoku2=0;scanyaz2=0;
   scanoku3=0;scanyaz3=0;
   scanoku4=0;scanyaz4=0;
   scanoku5=0;scanyaz5=0;
   scanoku6=0;scanyaz6=0;
   scanoku7=0;scanyaz7=0;



   for (int i=0;i<8;i++)
   {
       lastregisterpose(0,i)=9999;
       lastregisterpose(1,i)=9999;
       lastregisterpose(2,i)=9999;
       registerlaser[i]=0;
       islaserCallbackfinish[i]=true;
       gelenlaser[i]=0;
       islenenlaser[i]=0;
       okunanlaser[i]=0;
       lastregisterposescore[i]=0;
       isFirst[i]=true;

       activerobotid[i]=true;

       std::stringstream ss;
       ss << "base_frame" << i;
       baseframe = ss.str();


       if(!private_nh_.getParam(baseframe, p_base_frame_[i]))
       {
           p_base_frame_[i] = "";
           activerobotid[i]=false;
       }

       std::stringstream ss2;
       ss2 << "odom_frame" << i;
       odomframe = ss2.str();

       if(!private_nh_.getParam(odomframe, p_odom_frame_[i]))
       {
           p_odom_frame_[i] = "";
           activerobotid[i]=false;
       }

       std::stringstream ss3;
       ss3 << "scan_topic" << i;
       scantopic = ss3.str();

       if(!private_nh_.getParam(scantopic, p_scan_topic_[i]))
       {
           p_scan_topic_[i] = "";
           activerobotid[i]=false;
       }

       std::stringstream ss4;
       ss4 << "tf_map_scanmatch_transform_frame_name" << i;
       scanmatchframe = ss4.str();

       if(!private_nh_.getParam(scanmatchframe, p_tf_map_scanmatch_transform_frame_name_[i]))
       {
           p_tf_map_scanmatch_transform_frame_name_[i] = "";
           activerobotid[i]=false;
       }

       std::cout << p_base_frame_[i] << "  " << p_odom_frame_[i]  << "  "<< p_scan_topic_[i] <<  "  "<< p_tf_map_scanmatch_transform_frame_name_[i] <<std::endl;
   }


  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
      for (int i=0;i<8;i++)
      {
          std::stringstream ss5;
          ss5 << "scanmatch_odom" << i;
          if (activerobotid[i])
                odometryPublisher_[i] = node_.advertise<nav_msgs::Odometry>(ss5.str(), 50);
      }
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {

    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  //ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  //ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  //ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  //scanSubscriber_[0] = node_.subscribe(p_scan_topic_[0], p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);


  int rid=0;
  if (activerobotid[rid])
  {
      scanSubscriber_[0] = node_.subscribe(p_scan_topic_[0], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc0, this);
      laser_process_thr0 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc0, this, 1));
  }
  rid=1;
  if (activerobotid[rid])
  {
      scanSubscriber_[1] = node_.subscribe(p_scan_topic_[1], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc1, this);
      laser_process_thr1 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc1, this, 1));
  }
  rid=2;
  if (activerobotid[rid])
  {
      scanSubscriber_[2] = node_.subscribe(p_scan_topic_[2], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc2, this);
      laser_process_thr2 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc2, this, 1));
  }
  rid=3;
  if (activerobotid[rid])
  {
      scanSubscriber_[3] = node_.subscribe(p_scan_topic_[3], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc3, this);
      laser_process_thr3 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc3, this, 1));
  }
  rid=4;
  if (activerobotid[rid])
  {
      scanSubscriber_[4] = node_.subscribe(p_scan_topic_[4], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc4, this);
      laser_process_thr4 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc4, this, 1));
  }
  rid=5;
  if (activerobotid[rid])
  {
      scanSubscriber_[5] = node_.subscribe(p_scan_topic_[5], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc5, this);
      laser_process_thr5 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc5, this, 1));
  }
  rid=6;
  if (activerobotid[rid])
  {
      scanSubscriber_[6] = node_.subscribe(p_scan_topic_[6], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc6, this);
      laser_process_thr6 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc6, this, 1));
  }
  rid=7;
  if (activerobotid[rid])
  {
      scanSubscriber_[7] = node_.subscribe(p_scan_topic_[7], p_scan_subscriber_queue_size_, &HectorMappingRos::laser_callback_fnc7, this);
      laser_process_thr7 = new boost::thread(boost::bind(&HectorMappingRos::laser_process_fnc7, this, 1));
  }


  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);

  for (int i=0;i<8;i++)
  {
      std::stringstream ss6;
      ss6 << "slam_out_pose" << i;
      if (activerobotid[i])
            posePublisher_[i] = node_.advertise<geometry_msgs::PoseStamped>(ss6.str(), 1, false);
  }

  //posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);


  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;
  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  laser_register_thr = new boost::thread(boost::bind(&HectorMappingRos::laser_register_fnc, this, 1));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;

  if(laser_process_thr0)
    delete laser_process_thr0;
  if(laser_process_thr1)
    delete laser_process_thr1;
  if(laser_process_thr2)
    delete laser_process_thr2;
  if(laser_process_thr3)
    delete laser_process_thr3;
  if(laser_process_thr4)
    delete laser_process_thr4;
  if(laser_process_thr5)
    delete laser_process_thr5;
  if(laser_process_thr6)
    delete laser_process_thr6;
  if(laser_process_thr7)
    delete laser_process_thr7;

}

void HectorMappingRos::laser_callback_fnc0(const sensor_msgs::LaserScan& scan)
{
    int rid=0;

    if (gelenlaser[rid]<80)
        Scan0.push_back(scan);
    else
        Scan0.at(scanyaz0)=scan;

    gelenlaser[rid]++;
    scanyaz0=gelenlaser[rid]%80;

    std::cout  << islenenlaser[0] << "   " << islenenlaser[1] << "   " << islenenlaser[2] << "    " << registerlaser[0] << "    " << registerlaser[1] << "    " << registerlaser[2] << "    " << scan.header.stamp << std::endl;
}

void HectorMappingRos::laser_process_fnc0(int x)
{
    int rid=0;
    while (true)
    {
      try
      {
        while ( scanyaz0 == scanoku0)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz0-scanoku0>5 || ( scanyaz0<scanoku0 && (scanyaz0+80-scanoku0>5)))
            scanoku0=(scanoku0+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan0.at(scanoku0), laserScanContainer0[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan0.at(scanoku0).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan0.at(scanoku0).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

//                startEstimate[0]=10;startEstimate[1]=10;startEstimate[2]=0;

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer0[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses0(0,indx)=startEstimate[0];
                poses0(1,indx)=startEstimate[1];
                poses0(2,indx)=startEstimate[2];
                zaman[0][indx]=Scan0.at(scanoku0).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer0[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses0(0,indx)=tmp2(0,rid);
                poses0(1,indx)=tmp2(1,rid);
                poses0(2,indx)=tmp2(2,rid);
                zaman[0][indx]=Scan0.at(scanoku0).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku0=(scanoku0+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "0  " << e.what() << std::endl;

        }
    }
}





void HectorMappingRos::laser_callback_fnc1(const sensor_msgs::LaserScan& scan)
{   int rid=1;
    if (gelenlaser[rid]<80)
        Scan1.push_back(scan);
    else
        Scan1.at(scanyaz1)=scan;

    gelenlaser[rid]++;
    scanyaz1=gelenlaser[rid]%80;
}
void HectorMappingRos::laser_process_fnc1(int x)
{
    int rid=1;
    while (true)
    {
      try
      {
        while ( scanyaz1 == scanoku1)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz1-scanoku1>5 || ( scanyaz1<scanoku1 && (scanyaz1+80-scanoku1>5)))
            scanoku1=(scanoku1+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan1.at(scanoku1), laserScanContainer1[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan1.at(scanoku1).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan1.at(scanoku1).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

//                startEstimate[0]=-10;startEstimate[1]=-10;startEstimate[2]=0;

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer1[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses1(0,indx)=startEstimate[0];
                poses1(1,indx)=startEstimate[1];
                poses1(2,indx)=startEstimate[2];
                zaman[1][indx]=Scan1.at(scanoku1).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer1[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses1(0,indx)=tmp2(0,rid);
                poses1(1,indx)=tmp2(1,rid);
                poses1(2,indx)=tmp2(2,rid);
                zaman[1][indx]=Scan1.at(scanoku1).header.stamp;

                //std::cout << tmp(0,rid) << "  "<< tmp(1,rid) << "  " << tmp(2,rid) <<"   "<< tmp2(0,rid) << "  "<< tmp2(1,rid) << "  " << tmp2(2,rid) << std::endl;
            }
            islenenlaser[rid]++;
        }
        scanoku1=(scanoku1+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "1  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_callback_fnc2(const sensor_msgs::LaserScan& scan)
{   int rid=2;
    if (gelenlaser[rid]<80)
        Scan2.push_back(scan);
    else
        Scan2.at(scanyaz2)=scan;

    gelenlaser[rid]++;
    scanyaz2=gelenlaser[rid]%80;

}
void HectorMappingRos::laser_process_fnc2(int x)
{
    int rid=2;
    while (true)
    {
      try
      {
        while ( scanyaz2 == scanoku2)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz2-scanoku2>5 || ( scanyaz2<scanoku2 && (scanyaz2+80-scanoku2>5)))
            scanoku2=(scanoku2+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan2.at(scanoku2), laserScanContainer2[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan2.at(scanoku2).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan2.at(scanoku2).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer2[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses2(0,indx)=startEstimate[0];
                poses2(1,indx)=startEstimate[1];
                poses2(2,indx)=startEstimate[2];
                zaman[2][indx]=Scan2.at(scanoku2).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer2[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses2(0,indx)=tmp2(0,rid);
                poses2(1,indx)=tmp2(1,rid);
                poses2(2,indx)=tmp2(2,rid);
                zaman[2][indx]=Scan2.at(scanoku2).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku2=(scanoku2+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "2  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_callback_fnc3(const sensor_msgs::LaserScan& scan)
{   int rid=3;
    if (gelenlaser[rid]<80)
        Scan3.push_back(scan);
    else
        Scan3.at(scanyaz3)=scan;

    gelenlaser[rid]++;
    scanyaz3=gelenlaser[rid]%80;
}
void HectorMappingRos::laser_process_fnc3(int x)
{
    int rid=3;
    while (true)
    {
      try
      {
        while ( scanyaz3 == scanoku3)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz3-scanoku3>5 || ( scanyaz3<scanoku3 && (scanyaz3+80-scanoku3>5)))
            scanoku3=(scanoku3+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan3.at(scanoku3), laserScanContainer3[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan3.at(scanoku3).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan3.at(scanoku3).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer3[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses3(0,indx)=startEstimate[0];
                poses3(1,indx)=startEstimate[1];
                poses3(2,indx)=startEstimate[2];
                zaman[3][indx]=Scan3.at(scanoku3).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer3[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses3(0,indx)=tmp2(0,rid);
                poses3(1,indx)=tmp2(1,rid);
                poses3(2,indx)=tmp2(2,rid);
                zaman[3][indx]=Scan3.at(scanoku3).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku3=(scanoku3+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "3  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_callback_fnc4(const sensor_msgs::LaserScan& scan)
{   int rid=4;
    if (gelenlaser[rid]<80)
        Scan4.push_back(scan);
    else
        Scan4.at(scanyaz4)=scan;

    gelenlaser[rid]++;
    scanyaz4=gelenlaser[rid]%80;
}

void HectorMappingRos::laser_process_fnc4(int x)
{
    int rid=4;
    while (true)
    {
      try
      {
        while ( scanyaz4 == scanoku4)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz4-scanoku4>5 || ( scanyaz4<scanoku4 && (scanyaz4+80-scanoku4>5)))
            scanoku4=(scanoku4+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan4.at(scanoku4), laserScanContainer4[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan4.at(scanoku4).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan4.at(scanoku4).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer4[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses4(0,indx)=startEstimate[0];
                poses4(1,indx)=startEstimate[1];
                poses4(2,indx)=startEstimate[2];
                zaman[4][indx]=Scan4.at(scanoku4).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer4[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses4(0,indx)=tmp2(0,rid);
                poses4(1,indx)=tmp2(1,rid);
                poses4(2,indx)=tmp2(2,rid);
                zaman[4][indx]=Scan4.at(scanoku4).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku4=(scanoku4+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "4  " << e.what() << std::endl;

        }
    }
}


void HectorMappingRos::laser_callback_fnc5(const sensor_msgs::LaserScan& scan)
{   int rid=5;
    if (gelenlaser[rid]<80)
        Scan5.push_back(scan);
    else
        Scan5.at(scanyaz5)=scan;

    gelenlaser[rid]++;
    scanyaz5=gelenlaser[rid]%80;
}

void HectorMappingRos::laser_process_fnc5(int x)
{
    int rid=5;
    while (true)
    {
      try
      {
        while ( scanyaz5 == scanoku5)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz5-scanoku5>5 || ( scanyaz5<scanoku5 && (scanyaz5+80-scanoku5>5)))
            scanoku5=(scanoku5+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan5.at(scanoku5), laserScanContainer5[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan5.at(scanoku5).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan5.at(scanoku5).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer5[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses5(0,indx)=startEstimate[0];
                poses5(1,indx)=startEstimate[1];
                poses5(2,indx)=startEstimate[2];
                zaman[5][indx]=Scan5.at(scanoku5).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer5[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses5(0,indx)=tmp2(0,rid);
                poses5(1,indx)=tmp2(1,rid);
                poses5(2,indx)=tmp2(2,rid);
                zaman[5][indx]=Scan5.at(scanoku5).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku5=(scanoku5+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "5  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_callback_fnc6(const sensor_msgs::LaserScan& scan)
{   int rid=6;
    if (gelenlaser[rid]<80)
        Scan6.push_back(scan);
    else
        Scan6.at(scanyaz6)=scan;

    gelenlaser[rid]++;
    scanyaz6=gelenlaser[rid]%80;
}

void HectorMappingRos::laser_process_fnc6(int x)
{
    int rid=6;
    while (true)
    {
      try
      {
        while ( scanyaz6 == scanoku6)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz6-scanoku6>5 || ( scanyaz6<scanoku6 && (scanyaz6+80-scanoku6>5)))
            scanoku6=(scanoku6+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan6.at(scanoku6), laserScanContainer6[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan6.at(scanoku6).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan6.at(scanoku6).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer6[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses6(0,indx)=startEstimate[0];
                poses6(1,indx)=startEstimate[1];
                poses6(2,indx)=startEstimate[2];
                zaman[6][indx]=Scan6.at(scanoku6).header.stamp;
                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer6[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses6(0,indx)=tmp2(0,rid);
                poses6(1,indx)=tmp2(1,rid);
                poses6(2,indx)=tmp2(2,rid);
                zaman[6][indx]=Scan6.at(scanoku6).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku6=(scanoku6+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "6  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_callback_fnc7(const sensor_msgs::LaserScan& scan)
{   int rid=7;
    if (gelenlaser[rid]<80)
        Scan7.push_back(scan);
    else
        Scan7.at(scanyaz7)=scan;

    gelenlaser[rid]++;
    scanyaz7=gelenlaser[rid]%80;
}

void HectorMappingRos::laser_process_fnc7(int x)
{
    int rid=7;
    while (true)
    {
      try
      {
        while ( scanyaz7 == scanoku7)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        while ( scanyaz7-scanoku7>5 || ( scanyaz7<scanoku7 && (scanyaz7+80-scanoku7>5)))
            scanoku7=(scanoku7+1)%80;

        int indx=islenenlaser[rid]%80;

        if (rosLaserScanToDataContainer(Scan7.at(scanoku7), laserScanContainer7[indx],slamProcessor->getScaleToMap()))
        {

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
            if (isFirst[rid])
            {
                tf::StampedTransform stamped_pose;
                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], Scan7.at(scanoku7).header.stamp, ros::Duration(0.5));
                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], Scan7.at(scanoku7).header.stamp, stamped_pose);
                tfScalar yaw, pitch, roll;
                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
                startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

                std::cout << startEstimate << std::endl;
                std::cout << p_odom_frame_[rid]<< "   "<< p_base_frame_[rid] << std::endl;

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer7[indx],startEstimate,rid,true);
                //tfmutex.unlock();

                poses7(0,indx)=startEstimate[0];
                poses7(1,indx)=startEstimate[1];
                poses7(2,indx)=startEstimate[2];
                zaman[7][indx]=Scan7.at(scanoku7).header.stamp;

                isFirst[rid]=false;
            }
            else
            {
                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
                startEstimate[0]=tmp(0,rid);
                startEstimate[1]=tmp(1,rid);
                startEstimate[2]=tmp(2,rid);

                //tfmutex.lock();
                slamProcessor->update2(laserScanContainer7[indx],startEstimate,rid);
                //tfmutex.unlock();

                Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
                poses7(0,indx)=tmp2(0,rid);
                poses7(1,indx)=tmp2(1,rid);
                poses7(2,indx)=tmp2(2,rid);
                zaman[7][indx]=Scan7.at(scanoku7).header.stamp;
            }
            islenenlaser[rid]++;
        }
        scanoku7=(scanoku7+1)%80;
        }catch(std::exception& e)
        {
            std::cout << "7  " << e.what() << std::endl;

        }
    }
}

void HectorMappingRos::laser_register_fnc(int x)
{

    while (true)
    {
        while ( islenenlaser[0] == okunanlaser[0] && islenenlaser[1] == okunanlaser[1] && islenenlaser[2] == okunanlaser[2] && islenenlaser[3] == okunanlaser[3] && islenenlaser[4] == okunanlaser[4] && islenenlaser[5] == okunanlaser[5] && islenenlaser[6] == okunanlaser[6] && islenenlaser[7] == okunanlaser[7])
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        }
        int tmp[8],tmpid[8],selectedrobot=-1;
        double mxscore=-1;
        for (int i=0;i<8;i++)
        {
            tmpid[i]=-1;
            if (activerobotid[i] && islenenlaser[i] != okunanlaser[i] && islenenlaser[i]>0)
            {
                tmp[i]=islenenlaser[i];
                tmpid[i]=(islenenlaser[i]-1)%80;


                switch (i)
                {
                case 0:
                    lastregisterposescore[i]=(poses0(0,tmpid[i])-lastregisterpose(0,i))*(poses0(0,tmpid[i])-lastregisterpose(0,i))+(poses0(1,tmpid[i])-lastregisterpose(1,i))*(poses0(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses0(2,tmpid[i])-lastregisterpose(2,i))*(poses0(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 1:
                    lastregisterposescore[i]=(poses1(0,tmpid[i])-lastregisterpose(0,i))*(poses1(0,tmpid[i])-lastregisterpose(0,i))+(poses1(1,tmpid[i])-lastregisterpose(1,i))*(poses1(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses1(2,tmpid[i])-lastregisterpose(2,i))*(poses1(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 2:
                    lastregisterposescore[i]=(poses2(0,tmpid[i])-lastregisterpose(0,i))*(poses2(0,tmpid[i])-lastregisterpose(0,i))+(poses2(1,tmpid[i])-lastregisterpose(1,i))*(poses2(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses2(2,tmpid[i])-lastregisterpose(2,i))*(poses2(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 3:
                    lastregisterposescore[i]=(poses3(0,tmpid[i])-lastregisterpose(0,i))*(poses3(0,tmpid[i])-lastregisterpose(0,i))+(poses3(1,tmpid[i])-lastregisterpose(1,i))*(poses3(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses3(2,tmpid[i])-lastregisterpose(2,i))*(poses3(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 4:
                    lastregisterposescore[i]=(poses4(0,tmpid[i])-lastregisterpose(0,i))*(poses4(0,tmpid[i])-lastregisterpose(0,i))+(poses4(1,tmpid[i])-lastregisterpose(1,i))*(poses4(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses4(2,tmpid[i])-lastregisterpose(2,i))*(poses4(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 5:
                    lastregisterposescore[i]=(poses5(0,tmpid[i])-lastregisterpose(0,i))*(poses5(0,tmpid[i])-lastregisterpose(0,i))+(poses5(1,tmpid[i])-lastregisterpose(1,i))*(poses5(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses5(2,tmpid[i])-lastregisterpose(2,i))*(poses5(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 6:
                    lastregisterposescore[i]=(poses6(0,tmpid[i])-lastregisterpose(0,i))*(poses6(0,tmpid[i])-lastregisterpose(0,i))+(poses6(1,tmpid[i])-lastregisterpose(1,i))*(poses6(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses6(2,tmpid[i])-lastregisterpose(2,i))*(poses6(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                case 7:
                    lastregisterposescore[i]=(poses7(0,tmpid[i])-lastregisterpose(0,i))*(poses7(0,tmpid[i])-lastregisterpose(0,i))+(poses7(1,tmpid[i])-lastregisterpose(1,i))*(poses7(1,tmpid[i])-lastregisterpose(1,i))+ 10* (poses7(2,tmpid[i])-lastregisterpose(2,i))*(poses7(2,tmpid[i])-lastregisterpose(2,i));
                    break;
                }
                if (lastregisterposescore[i]>mxscore)
                {
                    mxscore=lastregisterposescore[i];
                    selectedrobot=i;
                }
            }
        }

        okunanlaser[selectedrobot]=tmp[selectedrobot];

        Eigen::Vector3f cpose(Eigen::Vector3f::Zero());
        switch (selectedrobot)
        {
            case 0:
                lastregisterpose(0,selectedrobot)=poses0(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses0(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses0(2,tmpid[selectedrobot]);
                cpose[0]=poses0(0,tmpid[selectedrobot]);
                cpose[1]=poses0(1,tmpid[selectedrobot]);
                cpose[2]=poses0(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer0[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 1:
                lastregisterpose(0,selectedrobot)=poses1(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses1(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses1(2,tmpid[selectedrobot]);
                cpose[0]=poses1(0,tmpid[selectedrobot]);
                cpose[1]=poses1(1,tmpid[selectedrobot]);
                cpose[2]=poses1(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer1[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 2:
                lastregisterpose(0,selectedrobot)=poses2(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses2(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses2(2,tmpid[selectedrobot]);
                cpose[0]=poses2(0,tmpid[selectedrobot]);
                cpose[1]=poses2(1,tmpid[selectedrobot]);
                cpose[2]=poses2(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer2[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 3:
                lastregisterpose(0,selectedrobot)=poses3(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses3(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses3(2,tmpid[selectedrobot]);
                cpose[0]=poses3(0,tmpid[selectedrobot]);
                cpose[1]=poses3(1,tmpid[selectedrobot]);
                cpose[2]=poses3(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer3[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 4:
                lastregisterpose(0,selectedrobot)=poses4(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses4(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses4(2,tmpid[selectedrobot]);
                cpose[0]=poses4(0,tmpid[selectedrobot]);
                cpose[1]=poses4(1,tmpid[selectedrobot]);
                cpose[2]=poses4(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer4[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 5:
                lastregisterpose(0,selectedrobot)=poses5(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses5(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses5(2,tmpid[selectedrobot]);
                cpose[0]=poses5(0,tmpid[selectedrobot]);
                cpose[1]=poses5(1,tmpid[selectedrobot]);
                cpose[2]=poses5(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer5[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 6:
                lastregisterpose(0,selectedrobot)=poses6(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses6(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses6(2,tmpid[selectedrobot]);
                cpose[0]=poses6(0,tmpid[selectedrobot]);
                cpose[1]=poses6(1,tmpid[selectedrobot]);
                cpose[2]=poses6(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer6[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
            case 7:
                lastregisterpose(0,selectedrobot)=poses7(0,tmpid[selectedrobot]);
                lastregisterpose(1,selectedrobot)=poses7(1,tmpid[selectedrobot]);
                lastregisterpose(2,selectedrobot)=poses7(2,tmpid[selectedrobot]);
                cpose[0]=poses7(0,tmpid[selectedrobot]);
                cpose[1]=poses7(1,tmpid[selectedrobot]);
                cpose[2]=poses7(2,tmpid[selectedrobot]);
                slamProcessor->update3(laserScanContainer7[tmpid[selectedrobot]],cpose,selectedrobot);
                break;
        }
        registerlaser[selectedrobot]++;

        Eigen::Matrix<float, 3, 8> Tmp= slamProcessor->getLastScanMatchPose();
        Eigen::Vector3f lastpose;
        lastpose[0]=Tmp(0,selectedrobot);
        lastpose[1]=Tmp(1,selectedrobot);
        lastpose[2]=Tmp(2,selectedrobot);
        Eigen::Matrix<float, 3, 24> Tmp2= slamProcessor->getLastScanMatchCovariance();
        Eigen::Matrix3f lastcov;
        for (int i=0;i<3;i++)
          for (int j=0;j<3;j++)
             lastcov(i,j)=Tmp2(i,selectedrobot*3+j);

        poseInfoContainer_[selectedrobot].update(lastpose, lastcov, zaman[selectedrobot][tmpid[selectedrobot]], p_map_frame_);
        tfB_->sendTransform( tf::StampedTransform(poseInfoContainer_[selectedrobot].getTfTransform(), zaman[selectedrobot][tmpid[selectedrobot]], p_map_frame_, p_tf_map_scanmatch_transform_frame_name_[selectedrobot]));


    }
}



//void HectorMappingRos::multiscanCallback(const sensor_msgs::LaserScan& scan,int rid)
//{

//  islaserCallbackfinish[rid]=false;
//  islenenlaser[rid]++;
//  if (rid==0)
//      std::cout << laser_point_cloud_[0].points.size() << "   " <<  islenenlaser[0] << "  " << islenenlaser[1] << "  "<< islenenlaser[2] << "  " << scan.header.stamp << std::endl;

//  double rt=0,r1=0,r2=0;
//  Eigen::Vector3f map_pose;

//  ros::WallTime startTime = ros::WallTime::now();

//    ros::Duration dur (0.5);

//    if (tf_.waitForTransform(p_base_frame_[rid],scan.header.frame_id, scan.header.stamp,dur))
//    {
//      tf::StampedTransform laserTransform;
//      tf_.lookupTransform(p_base_frame_[rid],scan.header.frame_id, scan.header.stamp, laserTransform);
//      projector_[rid].projectLaser(scan, laser_point_cloud_[rid],30.0);
//      Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
//      if(rosPointCloudToDataContainer(laser_point_cloud_[rid], laserTransform, laserScanContainer[rid], slamProcessor->getScaleToMap()))
//      {
//          try
//          {
//            tf::StampedTransform stamped_pose;

////            tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], scan.header.stamp, ros::Duration(0.5));
////            tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], scan.header.stamp, stamped_pose);
////            tfScalar yaw, pitch, roll;
////            stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
////            map_pose = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);
//           if (isFirst[rid])
//             {
//                tf_.waitForTransform(p_odom_frame_[rid],p_base_frame_[rid], scan.header.stamp, ros::Duration(0.5));
//                tf_.lookupTransform(p_odom_frame_[rid], p_base_frame_[rid], scan.header.stamp, stamped_pose);
//                tfScalar yaw, pitch, roll;
//                stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);
//                map_pose = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);

//                 prev_map_poses(0,rid)=map_pose[0];
//                 prev_map_poses(1,rid)=map_pose[1];
//                 prev_map_poses(2,rid)=map_pose[2];
//                 startEstimate[0]=map_pose[0];
//                 startEstimate[1]=map_pose[1];
//                 startEstimate[2]=map_pose[2];
//                 isFirst[rid]=false;
//                 slamProcessor->update(laserScanContainer[rid], startEstimate,rid,true);
//             }
//             else
//             {

//                Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
//                startEstimate[0]=tmp(0,rid);
//                startEstimate[1]=tmp(1,rid);
//                startEstimate[2]=tmp(2,rid);

////                 rt=sqrt( (map_pose[0]-prev_map_poses(0,rid))*(map_pose[0]-prev_map_poses(0,rid))+(map_pose[1]-prev_map_poses(1,rid))*(map_pose[1]-prev_map_poses(1,rid)));
////                 r1=atan2(map_pose[1]-prev_map_poses(1,rid),map_pose[0]-prev_map_poses(0,rid))-prev_map_poses(2,rid);
////                 r2=map_pose[2]-prev_map_poses(2,rid)-r1;
////                 startEstimate[0]=startEstimate[0]+rt*cos(startEstimate[2]+r1);
////                 startEstimate[1]=startEstimate[1]+rt*sin(startEstimate[2]+r1);
////                 startEstimate[2]=startEstimate[2]+r1+r2;

//                slamProcessor->update(laserScanContainer[rid], startEstimate,rid);
//             }

// //            prev_map_poses(0,rid)=map_pose[0];
// //            prev_map_poses(1,rid)=map_pose[1];
// //            prev_map_poses(2,rid)=map_pose[2];

//          }
//          catch(tf::TransformException e)
//          {
//            ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_[rid].c_str());
//            islaserCallbackfinish[rid]=true;
//            return;
////            Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
////            startEstimate[0]=tmp(0,rid);
////            startEstimate[1]=tmp(1,rid);
////            startEstimate[2]=tmp(2,rid);
//          }
//      }

//    }
//    else
//    {
//      ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_[rid].c_str(), scan.header.frame_id.c_str());
//      islaserCallbackfinish[rid]=true;
//      return;
//    }


////  if (p_timing_output_)
////  {
////    ros::WallDuration duration = ros::WallTime::now() - startTime;
////    ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
////  }

////  //If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
////  if (p_map_with_known_poses_)
////  {
////    return;
////  }
//    Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
//    Eigen::Vector3f lastpose;
//    lastpose[0]=tmp(0,rid);
//    lastpose[1]=tmp(1,rid);
//    lastpose[2]=tmp(2,rid);
//    Eigen::Matrix<float, 3, 24> tmp2= slamProcessor->getLastScanMatchCovariance();
//    Eigen::Matrix3f lastcov;
//    for (int i=0;i<3;i++)
//       for (int j=0;j<3;j++)
//          lastcov(i,j)=tmp2(i,rid*3+j);

//    poseInfoContainer_[rid].update(lastpose, lastcov, scan.header.stamp, p_map_frame_);

////  //poseUpdatePublisher_.publish(poseInfoContainer_[rid].getPoseWithCovarianceStamped());
////  posePublisher_[rid].publish(poseInfoContainer_[rid].getPoseStamped());

////  if(p_pub_odometry_)
////  {
////    nav_msgs::Odometry tmp;
////    tmp.pose = poseInfoContainer_[rid].getPoseWithCovarianceStamped().pose;

////    tmp.header = poseInfoContainer_[rid].getPoseWithCovarianceStamped().header;
////    odometryPublisher_[rid].publish(tmp);
////  }
////  if (p_pub_map_odom_transform_)
////  {
////    tf::StampedTransform odom_to_base;
////    try
////    {
////      tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
////      tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
////    }
////    catch(tf::TransformException e)
////    {
////      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
////      odom_to_base.setIdentity();
////    }
////    map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
////    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
////  }

////  if (p_pub_map_scanmatch_transform_)
////  {
////      tfmutex.lock();
////      tfB_->sendTransform( tf::StampedTransform(poseInfoContainer_[rid].getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_[rid]));
////      tfmutex.unlock();
////  }

//  islaserCallbackfinish[rid]=true;
//}




//void HectorMappingRos::mscanCallback(const sensor_msgs::LaserScan& scan)
//{
//  int rid=0;
//  islaserCallbackfinish[rid]=false;
//  int indx=islenenlaser[rid]%80;

//  std::cout <<  islenenlaser[0] << "  " << islenenlaser[1] << "  "<< islenenlaser[2] << "  " << scan.header.stamp << std::endl;



//  if (rosLaserScanToDataContainer(scan, laserScanContainer[indx],slamProcessor->getScaleToMap()))
//  {
//      Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());
//      if (isFirst[rid])
//      {
//          isFirst[rid]=false;
//          startEstimate[0]=10;
//          startEstimate[1]=10;
//          startEstimate[2]=0;
//          slamProcessor->update2(laserScanContainer[indx],startEstimate,rid,true);
//          poses(0,indx)=10;
//          poses(1,indx)=10;
//          poses(2,indx)=0;

//      }
//      else
//      {
//          Eigen::Matrix<float, 3, 8> tmp= slamProcessor->getLastScanMatchPose();
//          startEstimate[0]=tmp(0,rid);
//          startEstimate[1]=tmp(1,rid);
//          startEstimate[2]=tmp(2,rid);

//          slamProcessor->update2(laserScanContainer[indx],startEstimate,rid);

//          Eigen::Matrix<float, 3, 8> tmp2= slamProcessor->getLastScanMatchPose();
//          poses(0,indx)=tmp2(0,rid);
//          poses(1,indx)=tmp2(1,rid);
//          poses(2,indx)=tmp2(2,rid);
//      }
//      islenenlaser[rid]++;
//  }

//  islaserCallbackfinish[rid]=true;
//}

//void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
//{
//  if (hectorDrawings)
//  {
//    hectorDrawings->setTime(scan.header.stamp);
//  }

//  ros::WallTime startTime = ros::WallTime::now();

//  if (!p_use_tf_scan_transformation_)
//  {
//    if (rosLaserScanToDataContainer(scan, laserScanContainer[0],slamProcessor->getScaleToMap()))
//    {
//      slamProcessor->update(laserScanContainer[0],slamProcessor->getLastScanMatchPose());
//    }
//  }
//  else
//  {
//    ros::Duration dur (0.5);

//    if (tf_.waitForTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp,dur))
//    {
//      tf::StampedTransform laserTransform;
//      tf_.lookupTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp, laserTransform);

//      //projector_.transformLaserScanToPointCloud(p_base_frame_ ,scan, pointCloud,tf_);
//      projector_.projectLaser(scan, laser_point_cloud_,30.0);

//      if (scan_point_cloud_publisher_.getNumSubscribers() > 0){
//        scan_point_cloud_publisher_.publish(laser_point_cloud_);
//      }

//      Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

//      if(rosPointCloudToDataContainer(laser_point_cloud_, laserTransform, laserScanContainer, slamProcessor->getScaleToMap()))
//      {
//        if (initial_pose_set_){
//          initial_pose_set_ = false;
//          startEstimate = initial_pose_;
//        }else if (p_use_tf_pose_start_estimate_){

//          try
//          {
//            tf::StampedTransform stamped_pose;

//            tf_.waitForTransform(p_map_frame_,p_base_frame_, scan.header.stamp, ros::Duration(0.5));
//            tf_.lookupTransform(p_map_frame_, p_base_frame_,  scan.header.stamp, stamped_pose);

//            tfScalar yaw, pitch, roll;
//            stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);

//            startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);
//          }
//          catch(tf::TransformException e)
//          {
//            ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
//            startEstimate = slamProcessor->getLastScanMatchPose();
//          }

//        }else{
//          startEstimate = slamProcessor->getLastScanMatchPose();
//        }


//        if (p_map_with_known_poses_){
//          slamProcessor->update(laserScanContainer, startEstimate, true);
//        }else{
//          slamProcessor->update(laserScanContainer, startEstimate);
//        }
//      }

//    }else{
//      ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
//      return;
//    }
//  }

//  if (p_timing_output_)
//  {
//    ros::WallDuration duration = ros::WallTime::now() - startTime;
//    ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
//  }

//  //If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
//  if (p_map_with_known_poses_)
//  {
//    return;
//  }

//  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

//  poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
//  posePublisher_.publish(poseInfoContainer_.getPoseStamped());

//  if(p_pub_odometry_)
//  {
//    nav_msgs::Odometry tmp;
//    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

//    tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
//    odometryPublisher_.publish(tmp);
//  }

//  if (p_pub_map_odom_transform_)
//  {
//    tf::StampedTransform odom_to_base;

//    try
//    {
//      tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
//      tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
//    }
//    catch(tf::TransformException e)
//    {
//      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
//      odom_to_base.setIdentity();
//    }
//    map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
//    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
//  }

//  if (p_pub_map_scanmatch_transform_){
//    tfB_->sendTransform( tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_[0]));
//  }
//}


void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }

  return true;
}

bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }

  return true;
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);
  int map_size_x = map.info.width;
  int map_size_y = map.info.height;
  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  initial_pose_set_ = true;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  initial_pose_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(pose.getRotation()));
  ROS_INFO("Setting initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}


