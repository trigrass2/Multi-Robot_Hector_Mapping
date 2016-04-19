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


#ifndef HECTOR_MAPPING_ROS_H__
#define HECTOR_MAPPING_ROS_H__

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>

#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/GetMap.h"

#include "slam_main/HectorSlamProcessor.h"

#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"

#include <boost/thread.hpp>

#include "PoseInfoContainer.h"


class HectorDrawings;
class HectorDebugInfoProvider;

class MapPublisherContainer
{
public:
  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos
{
public:
  HectorMappingRos();
  ~HectorMappingRos();

  int sayac;

  void laser_callback_fnc0(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc1(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc2(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc3(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc4(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc5(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc6(const sensor_msgs::LaserScan& scan);
  void laser_callback_fnc7(const sensor_msgs::LaserScan& scan);

  void laser_process_fnc0(int x);
  void laser_process_fnc1(int x);
  void laser_process_fnc2(int x);
  void laser_process_fnc3(int x);
  void laser_process_fnc4(int x);
  void laser_process_fnc5(int x);
  void laser_process_fnc6(int x);
  void laser_process_fnc7(int x);

  void laser_register_fnc(int x);


  void scanCallback(const sensor_msgs::LaserScan& scan);
  void multiscanCallback(const sensor_msgs::LaserScan& scan,int rid);
  void mscanCallback(const sensor_msgs::LaserScan& scan);//,int rid);
  void mscanCallback1(const sensor_msgs::LaserScan& scan);//,int rid);


  void multiscanCallback1(const sensor_msgs::LaserScan& scan,int rid);
  void sysMsgCallback(const std_msgs::String& string);

  bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

  void publishMap(MapPublisherContainer& map_, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex = 0);

  bool rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap);
  bool rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap);

  void setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap);

  void publishTransformLoop(double p_transform_pub_period_);
  void publishMapLoop(double p_map_pub_period_);
  void publishTransform();

  void staticMapCallback(const nav_msgs::OccupancyGrid& map);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /*
  void setStaticMapData(const nav_msgs::OccupancyGrid& map);
  */
protected:  

  std::vector<sensor_msgs::LaserScan> Scan0;
  std::vector<sensor_msgs::LaserScan> Scan1;
  std::vector<sensor_msgs::LaserScan> Scan2;
  std::vector<sensor_msgs::LaserScan> Scan3;
  std::vector<sensor_msgs::LaserScan> Scan4;
  std::vector<sensor_msgs::LaserScan> Scan5;
  std::vector<sensor_msgs::LaserScan> Scan6;
  std::vector<sensor_msgs::LaserScan> Scan7;

  int bekleme;
  int scanoku0,scanyaz0;
  int scanoku1,scanyaz1;
  int scanoku2,scanyaz2;
  int scanoku3,scanyaz3;
  int scanoku4,scanyaz4;
  int scanoku5,scanyaz5;
  int scanoku6,scanyaz6;
  int scanoku7,scanyaz7;


  boost::thread* laser_process_thr0;
  boost::thread* laser_process_thr1;
  boost::thread* laser_process_thr2;
  boost::thread* laser_process_thr3;
  boost::thread* laser_process_thr4;
  boost::thread* laser_process_thr5;
  boost::thread* laser_process_thr6;
  boost::thread* laser_process_thr7;

  boost::thread* laser_register_thr;

  boost::mutex tfmutex;

  int okuindex;
  int gelenlaser[8];
  int islenenlaser[8];
  int registerlaser[8];
  int okunanlaser[8];
  double lastregisterposescore[8];
  bool islaserCallbackfinish[8];
  bool isFirst[8];
  Eigen::Vector3f prev_map_pose;
  Eigen::Matrix<float, 3, 8> prev_map_poses;
  Eigen::Vector3f robot_map_pose;

  HectorDebugInfoProvider* debugInfoProvider;
  HectorDrawings* hectorDrawings;

  int lastGetMapUpdateIndex;

  ros::NodeHandle node_;

  ros::Subscriber scanSubscriber_[8];
  ros::Subscriber sysMsgSubscriber_;

  ros::Subscriber mapSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
  tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;

  ros::Publisher posePublisher_[8];
  ros::Publisher poseUpdatePublisher_;
  ros::Publisher twistUpdatePublisher_;
  ros::Publisher odometryPublisher_[8];
  ros::Publisher scan_point_cloud_publisher_;//[8];

  std::vector<MapPublisherContainer> mapPubContainer;

  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;

  laser_geometry::LaserProjection projector_[8];

  tf::Transform map_to_odom_;

  boost::thread* map__publish_thread_;

  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::DataContainer laserScanContainer0[80];
  hectorslam::DataContainer laserScanContainer1[80];
  hectorslam::DataContainer laserScanContainer2[80];
  hectorslam::DataContainer laserScanContainer3[80];
  hectorslam::DataContainer laserScanContainer4[80];
  hectorslam::DataContainer laserScanContainer5[80];
  hectorslam::DataContainer laserScanContainer6[80];
  hectorslam::DataContainer laserScanContainer7[80];
  Eigen::Matrix<float, 3, 80> poses0;
  Eigen::Matrix<float, 3, 80> poses1;
  Eigen::Matrix<float, 3, 80> poses2;
  Eigen::Matrix<float, 3, 80> poses3;
  Eigen::Matrix<float, 3, 80> poses4;
  Eigen::Matrix<float, 3, 80> poses5;
  Eigen::Matrix<float, 3, 80> poses6;
  Eigen::Matrix<float, 3, 80> poses7;

  ros::Time zaman[8][80];

  Eigen::Matrix<float, 3, 8> lastregisterpose;

  PoseInfoContainer poseInfoContainer_[8];

  sensor_msgs::PointCloud laser_point_cloud_[8];

  ros::Time lastMapPublishTime;
  ros::Time lastScanTime;
  Eigen::Vector3f lastSlamPose;

  bool initial_pose_set_;
  Eigen::Vector3f initial_pose_;


  //-----------------------------------------------------------
  // Parameters
  bool activerobotid[8];
  std::string p_base_frame_[8];
  std::string p_map_frame_;
  std::string p_odom_frame_[8];

  //Parameters related to publishing the scanmatcher pose directly via tf
  bool p_pub_map_scanmatch_transform_;
  std::string p_tf_map_scanmatch_transform_frame_name_[8];

  std::string p_scan_topic_[8];
  std::string p_sys_msg_topic_;

  std::string p_pose_update_topic_;
  std::string p_twist_update_topic_;

  bool p_pub_drawings;
  bool p_pub_debug_output_;
  bool p_pub_map_odom_transform_;
  bool p_pub_odometry_;
  bool p_advertise_map_service_;
  int p_scan_subscriber_queue_size_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;
  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_map_pub_period_;

  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_map_with_known_poses_;
  bool p_timing_output_;


  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;
};

#endif
