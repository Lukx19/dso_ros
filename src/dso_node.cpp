#include <dso_ros/dso_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dso_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dso_ros::DsoNode node(nh, nh_private);
  node.run();
  return 0;
}

dso_ros::DsoNode::DsoNode(ros::NodeHandle &n, ros::NodeHandle &n_private)
  : nh_(n)
  , nh_private_(n_private)
  , full_system_(new dso::FullSystem())
  , undistorter_()
  , frame_ID_(0)
  , calib_file_("")
  , vignette_file_("")
  , gamma_file_("")
  , image_sub_()
{
  image_sub_ = n.subscribe<sensor_msgs::Image>(
      "image", 1, boost::bind(&dso_ros::DsoNode::imageCb, this, _1));
  initParams();

  undistorter_.reset(dso::Undistort::getUndistorterForFile(
      calib_file_, gamma_file_, vignette_file_));
  dso::setGlobalCalib((int)undistorter_->getSize()[0],
                      (int)undistorter_->getSize()[1],
                      undistorter_->getK().cast<float>());
  initIOWrappers();
  reset();
}

dso_ros::DsoNode::~DsoNode()
{
  for (dso::IOWrap::Output3DWrapper *ow : full_system_->outputWrapper) {
    ow->join();
    delete ow;
  }
}

void dso_ros::DsoNode::imageCb(const sensor_msgs::ImageConstPtr &img)
{
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  assert(cv_ptr->image.type() == CV_8U);
  assert(cv_ptr->image.channels() == 1);

  if (dso::setting_fullResetRequested) {
    ROS_ERROR_STREAM("[DSO_ROS]: System reset requested from DSO");
    reset();
  }

  dso::MinimalImageB min_img((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,
                             (unsigned char *)cv_ptr->image.data);
  std::unique_ptr<dso::ImageAndExposure> undist_img(
      undistorter_->undistort<unsigned char>(&min_img, 1, 0, 1.0f));
  full_system_->addActiveFrame(undist_img.get(), frame_ID_);
  ++frame_ID_;
}

void dso_ros::DsoNode::initParams()
{
  dso::setting_desiredImmatureDensity = 1000;
  dso::setting_desiredPointDensity = 1200;
  dso::setting_minFrames = 5;
  dso::setting_maxFrames = 7;
  dso::setting_maxOptIterations = 4;
  dso::setting_minOptIterations = 1;
  dso::setting_logStuff = false;
  dso::setting_kfGlobalWeight = 1.3;

  printf("MODE WITH CALIBRATION, but without exposure times!\n");
  dso::setting_photometricCalibration = 2;
  dso::setting_affineOptModeA = 0;
  dso::setting_affineOptModeB = 0;

  bool debug = nh_private_.param<bool>("debug", false);
  if (debug) {
    dso::setting_debugout_runquiet = true;
    dso::setting_logStuff = true;
  } else {
    dso::setting_debugout_runquiet = false;
    dso::setting_logStuff = false;
  }
  nh_private_.param<bool>("display_GUI", display_GUI_, false);
  nh_private_.param<std::string>("calib_file_path", calib_file_, "");
  nh_private_.param<std::string>("vignette_file_path", vignette_file_, "");
  nh_private_.param<std::string>("gamma_file_path", gamma_file_, "");
}

void dso_ros::DsoNode::initIOWrappers()
{
  if (display_GUI_) {
    full_system_->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(
        (int)undistorter_->getSize()[0], (int)undistorter_->getSize()[1]));
  }
  full_system_->outputWrapper.push_back(
      new dso_ros::ROSOutputWrapper(nh_, nh_private_));
}

void dso_ros::DsoNode::reset()
{
  std::vector<dso::IOWrap::Output3DWrapper *> wraps =
      full_system_->outputWrapper;

  for (dso::IOWrap::Output3DWrapper *ow : wraps)
    ow->reset();

  full_system_.reset(new dso::FullSystem());
  full_system_->linearizeOperation = false;
  full_system_->outputWrapper = wraps;
  if (undistorter_->photometricUndist != 0)
    full_system_->setGammaFunction(undistorter_->photometricUndist->getG());
  dso::setting_fullResetRequested = false;
}
