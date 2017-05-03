#ifndef DSO_ROS_DSO_NODE
#define DSO_ROS_DSO_NODE

#include <IOWrapper/Output3DWrapper.h>
#include <IOWrapper/Pangolin/PangolinDSOViewer.h>

#include <FullSystem/FullSystem.h>
#include <FullSystem/HessianBlocks.h>
#include <util/FrameShell.h>
#include <util/MinimalImage.h>
#include <util/Undistort.h>
#include <util/settings.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <dso_ros/ros_output_wrapper.h>
#include <boost/bind.hpp>

#include <Eigen/Dense>

namespace dso_ros
{
class DsoNode
{
public:
  DsoNode(ros::NodeHandle& n, ros::NodeHandle& n_private);

  virtual ~DsoNode();

  void imageCb(const sensor_msgs::ImageConstPtr& img);
  void run()
  {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::unique_ptr<dso::FullSystem> full_system_;
  std::unique_ptr<dso::Undistort> undistorter_;

  size_t frame_ID_;

  std::string calib_file_;
  std::string vignette_file_;
  std::string gamma_file_;

  ros::Subscriber image_sub_;

  bool display_GUI_;

  void initParams();
  void initIOWrappers();
  void reset();
};
}
#endif  // DSO_ROS_DSO_NODE
