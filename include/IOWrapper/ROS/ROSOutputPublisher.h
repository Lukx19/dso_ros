/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Eigen/Core"
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace dso
{
class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{
class ROSOutputPublisher : public Output3DWrapper
{
private:
  /* TODO publish dso pointcloud */

  /* camera frame id */
  std::string dso_frame_id;
  std::string camera_frame_id;
  std::string odom_frame_id;
  std::string base_frame_id;
  /* dso odometry_msg */
  ros::Publisher dso_odom_pub;

  tf::TransformListener tf_list;
  tf::TransformBroadcaster tf_broadcast;

public:
  explicit ROSOutputPublisher(ros::NodeHandle dso_node);

  virtual ~ROSOutputPublisher();

  virtual void publishGraph(const std::map<long, Eigen::Vector2i>& connectivity);

  virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final,
                                CalibHessian* HCalib);

  virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib);

  virtual void pushLiveFrame(FrameHessian* image);

  virtual void pushDepthImage(MinimalImageB3* image);

  virtual bool needPushDepthImage();

  virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF);
};
}
}
