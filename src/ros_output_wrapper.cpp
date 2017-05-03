#include <dso_ros/ros_output_wrapper.h>

using namespace dso_ros;

ROSOutputWrapper::ROSOutputWrapper(ros::NodeHandle& n,
                                   ros::NodeHandle& n_private)
{
  ROS_INFO("ROSOutputPublisher created\n");
  if (!n_private.hasParam("dso_frame_id")) {
    ROS_INFO("No param named world_frame found!");
  }
  if (!n_private.hasParam("camera_frame_id")) {
    ROS_INFO("No param named camera_frame found!");
  }
  n_private.param<std::string>("dso_frame_id", dso_frame_id_, "dso_odom");
  n_private.param<std::string>("camera_frame_id", camera_frame_id_, "camera");
  n_private.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  n_private.param<std::string>("base_frame_id", odom_frame_id_, "base_link");
  ROS_INFO_STREAM("world_frame = " << dso_frame_id_ << "\n");
  ROS_INFO_STREAM("camera_frame = " << camera_frame_id_ << "\n");
  dso_frame_id_ = "dso_odom";
  dso_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 5, false);
  //  dso_depht_image_pub_ = n.advertise<>s
}

ROSOutputWrapper::~ROSOutputWrapper()
{
  ROS_INFO("ROSOutputPublisher destroyed\n");
}

void ROSOutputWrapper::publishGraph(
    const std::map<long, Eigen::Vector2i>& connectivity)
{
  printf("OUT: got graph with %d edges\n", (int)connectivity.size());

  int maxWrite = 5;

  for (const std::pair<long, Eigen::Vector2i>& p : connectivity) {
    int idHost = p.first >> 32;
    int idTarget = p.first & 0xFFFFFFFF;
    printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n",
           idHost, idTarget, p.second[0], p.second[1]);
    maxWrite--;
    if (maxWrite == 0)
      break;
  }
}

void ROSOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian*>& frames,
                                        bool final, dso::CalibHessian* HCalib)
{
  for (dso::FrameHessian* f : frames) {
    printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d "
           "immature points. CameraToWorld:\n",
           f->frameID, final ? "final" : "non-final", f->shell->incoming_id,
           f->shell->timestamp, (int)f->pointHessians.size(),
           (int)f->pointHessiansMarginalized.size(),
           (int)f->immaturePoints.size());
    std::cout << f->shell->camToWorld.matrix3x4() << "\n";

    int maxWrite = 5;
    for (dso::PointHessian* p : f->pointHessians) {
      printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. "
             "%f, %d inlier-residuals\n",
             p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian),
             p->numGoodResiduals);
      maxWrite--;
      if (maxWrite == 0)
        break;
    }
  }
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell* frame,
                                      dso::CalibHessian* HCalib)
{
  tf::StampedTransform tf_odom_base;
  tf::StampedTransform tf_base_cam;
  try {
    tf_list_.waitForTransform(odom_frame_id_, base_frame_id_, ros::Time::now(),
                              ros::Duration(10.0));
    tf_list_.lookupTransform(odom_frame_id_, base_frame_id_, ros::Time::now(),
                             tf_odom_base);

    tf_list_.waitForTransform(base_frame_id_, camera_frame_id_,
                              ros::Time::now(), ros::Duration(10.0));
    tf_list_.lookupTransform(base_frame_id_, camera_frame_id_, ros::Time::now(),
                             tf_base_cam);
  } catch (...) {
    ROS_ERROR_STREAM("DSO_ROS: Not sucessfull in retrieving tf tranform from "
                     << odom_frame_id_ << "->" << camera_frame_id_);
    return;
  }

  /*
   * This function broadcasts tf transformation:
   * world->cam based on frame->camToWorld.matrix3x4()
   *
   * frame->camToWorld.matrix3x4() returns:
   *
   * m00 m01 m02 m03
   * m10 m11 m12 m13
   * m20 m21 m22 m23
   *
   * last column is translation vector
   * 3x3 matrix with diagonal m00 m11 and m22 is a rotation matrix
  */

  const Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> m =
      frame->camToWorld.matrix3x4();
  /* camera position */
  double camX = m(0, 3);
  double camY = m(1, 3);
  double camZ = m(2, 3);

  /* camera orientation */
  /* http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
   */
  double numX = 1 + m(0, 0) - m(1, 1) - m(2, 2);
  double numY = 1 - m(0, 0) + m(1, 1) - m(2, 2);
  double numZ = 1 - m(0, 0) - m(1, 1) + m(2, 2);
  double numW = 1 + m(0, 0) + m(1, 1) + m(2, 2);
  double camSX = sqrt(std::max(0.0, numX)) / 2;
  double camSY = sqrt(std::max(0.0, numY)) / 2;
  double camSZ = sqrt(std::max(0.0, numZ)) / 2;
  double camSW = sqrt(std::max(0.0, numW)) / 2;

  /* broadcast map -> cam_pose transformation */
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(camX, camY, camZ));
  tf::Quaternion q = tf::Quaternion(camSX, camSY, camSZ, camSW);

  transform.setRotation(q);
  tf::Transform tf_dso_base = transform * tf_base_cam.inverse();
  tf::Transform tf_dso_odom = tf_dso_base * tf_odom_base.inverse();
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                        dso_frame_id_, odom_frame_id_));

  ROS_INFO_STREAM("ROSOutputPublisher:" << base_frame_id_ << "->"
                                        << dso_frame_id_ << " tf broadcasted");
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = dso_frame_id_;
  tf::poseTFToMsg(tf_dso_base, odom.pose.pose);
  dso_odom_pub_.publish(odom);
  /* testing output */
  /*
        std::cout << frame->camToWorld.matrix3x4() << "\n";
  */
}

void ROSOutputWrapper::pushLiveFrame(dso::FrameHessian* image)
{
  // can be used to get the raw image / intensity pyramid.
}

void ROSOutputWrapper::pushDepthImage(dso::MinimalImageB3* image)
{
  // can be used to get the raw image with depth overlay.
}
bool ROSOutputWrapper::needPushDepthImage()
{
  return true;
}

void ROSOutputWrapper::pushDepthImageFloat(dso::MinimalImageF* image,
                                           dso::FrameHessian* KF)
{
  printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID "
         "%d). CameraToWorld:\n",
         KF->frameID, KF->shell->incoming_id, KF->shell->timestamp,
         KF->shell->id);
  std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

  int maxWrite = 5;
  for (int y = 0; y < image->h; y++) {
    for (int x = 0; x < image->w; x++) {
      if (image->at(x, y) <= 0)
        continue;

      printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x, y,
             image->at(x, y));
      maxWrite--;
      if (maxWrite == 0)
        break;
    }
    if (maxWrite == 0)
      break;
  }
}
