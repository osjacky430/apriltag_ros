/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include <sstream>

#include "apriltag_ros/apriltag_wrapper.hpp"
#include "apriltag_ros/common_functions.h"
#include "image_geometry/pinhole_camera_model.h"

#include "common/homography.h"

using namespace std::string_literals;

namespace {

inline auto get_apriltag_detector_param(ros::NodeHandle const& t_nh) {
  using namespace apriltag_ros;

  ApriltagDetectorParam ret_val;
  ret_val.quad_decimate_ = static_cast<float>(getAprilTagOption<double>(t_nh, "tag_decimate"s, 1.0));
  ret_val.quad_sigma_    = static_cast<float>(getAprilTagOption<double>(t_nh, "tag_blur"s, 0.0));
  ret_val.debug_         = getAprilTagOption<int>(t_nh, "tag_debug"s, 0);
  ret_val.refine_edges_  = getAprilTagOption<int>(t_nh, "tag_refine_edges"s, 1);
  ret_val.thread_count_  = getAprilTagOption<int>(t_nh, "tag_threads"s, 4);

  return ret_val;
}

}  // namespace

namespace apriltag_ros {

TagDetector::TagDetector(ros::NodeHandle const& pnh)
  : tf_(make_apriltag_family(getAprilTagOption<std::string>(pnh, "tag_family"s, "tag36h11"s))),
    td_(make_apriltag_detector(::get_apriltag_detector_param(pnh))),
    publish_tf_(getAprilTagOption<bool>(pnh, "publish_tf"s, false)) {
  // Parse standalone tag descriptions specified by user (stored on ROS
  // parameter server)
  XmlRpc::XmlRpcValue standalone_tag_descriptions;
  if (!pnh.getParam("standalone_tags"s, standalone_tag_descriptions)) {
    ROS_WARN("No april tags specified");
  } else {
    try {
      standalone_tag_descriptions_ = parseStandaloneTags(standalone_tag_descriptions);
    } catch (XmlRpc::XmlRpcException& e) {
      // in case any of the asserts in parseStandaloneTags() fail
      ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage());
    }
  }

  // parse tag bundle descriptions specified by user (stored on ROS parameter
  // server)
  XmlRpc::XmlRpcValue tag_bundle_descriptions;
  if (!pnh.getParam("tag_bundles"s, tag_bundle_descriptions)) {
    ROS_WARN("No tag bundles specified");
  } else {
    try {
      tag_bundle_descriptions_ = parseTagBundles(tag_bundle_descriptions);
    } catch (XmlRpc::XmlRpcException& e) {
      // In case any of the asserts in parseStandaloneTags() fail
      ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage());
    }
  }

  // Optionally remove duplicate detections in scene. Defaults to removing
  if (!pnh.getParam("remove_duplicates"s, remove_duplicates_)) {
    ROS_WARN("remove_duplicates parameter not provided. Defaulting to true");
    remove_duplicates_ = true;
  }

  // Tunable, but really, 2 is a good choice. Values of >=3
  // consume prohibitively large amounts of memory, and otherwise
  // you want the largest value possible.
  apriltag_detector_add_family_bits(td_.get(), tf_.get(), getAprilTagOption<int>(pnh, "max_hamming_dist"s, 2));

  // Get tf frame name to use for the camera
  if (!pnh.getParam("camera_frame"s, camera_tf_frame_)) {
    ROS_WARN_STREAM("Camera frame not specified, using 'camera'");
    camera_tf_frame_ = "camera"s;
  }
}

AprilTagDetectionArray TagDetector::detectTags(const cv_bridge::CvImageConstPtr& image,
                                               const sensor_msgs::CameraInfoConstPtr& camera_info) {
  // Convert image to AprilTag code's format
  cv::Mat gray_image;
  if (image->image.channels() == 1) {
    gray_image = image->image;
  } else {
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
  }
  image_u8_t apriltag_image{gray_image.cols, gray_image.rows, gray_image.cols, gray_image.data};

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  // Get camera intrinsic properties for rectified image.
  double const fx = camera_model.fx();  // focal length in camera x-direction [px]
  double const fy = camera_model.fy();  // focal length in camera y-direction [px]
  double const cx = camera_model.cx();  // optical center x-coordinate [px]
  double const cy = camera_model.cy();  // optical center y-coordinate [px]

  // Run AprilTag 2 algorithm on the image
  detections_ = apriltag_detection_wrapper_fn(td_.get(), &apriltag_image);

  // If remove_dulpicates_ is set to true, then duplicate tags are not allowed.
  // Thus any duplicate tag IDs visible in the scene must include at least 1
  // erroneous detection. Remove any tags with duplicate IDs to ensure removal
  // of these erroneous detections
  if (remove_duplicates_) {
    removeDuplicates();
  }

  // Compute the estimated translation and rotation individually for each
  // detected tag
  AprilTagDetectionArray tag_detection_array;
  std::vector<std::string> detection_names;
  tag_detection_array.header = image->header;
  std::map<std::string, std::vector<cv::Point3d> > bundleObjectPoints;
  std::map<std::string, std::vector<cv::Point2d> > bundleImagePoints;
  for (int i = 0; i < zarray_size(detections_.get()); i++) {
    // Get the i-th detected tag
    apriltag_detection_t* detection = nullptr;
    zarray_get(detections_.get(), i, &detection);

    // Bootstrap this for loop to find this tag's description amongst
    // the tag bundles. If found, add its points to the bundle's set of
    // object-image corresponding points (tag corners) for cv::solvePnP.
    // Don't yet run cv::solvePnP on the bundles, though, since we're still in
    // the process of collecting all the object-image corresponding points
    int const tagID        = detection->id;
    bool is_part_of_bundle = false;
    for (auto const& bundle : tag_bundle_descriptions_) {
      // Iterate over the registered bundles
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) {
        // This detected tag belongs to the j-th tag bundle (its ID was found in
        // the bundle description)
        is_part_of_bundle            = true;
        std::string const bundleName = bundle.name();

        //===== Corner points in the world frame coordinates
        double const s = bundle.memberSize(tagID) / 2;
        addObjectPoints(s, bundle.memberT_oi(tagID), bundleObjectPoints[bundleName]);

        //===== Corner points in the image frame coordinates
        addImagePoints(detection, bundleImagePoints[bundleName]);
      }
    }

    // Find this tag's description amongst the standalone tags
    // Print warning when a tag was found that is neither part of a
    // bundle nor standalone (thus it is a tag in the environment
    // which the user specified no description for, or Apriltags
    // misdetected a tag (bad ID or a false positive)).
    StandaloneTagDescription* standaloneDescription = nullptr;
    if (!findStandaloneTagDescription(tagID, standaloneDescription, !is_part_of_bundle)) {
      continue;
    }

    //=================================================================
    // The remainder of this for loop is concerned with standalone tag
    // poses!
    double const tag_size = standaloneDescription->size();

    // Get estimated tag pose in the camera frame.
    //
    // Note on frames:
    // The raw AprilTag 2 uses the following frames:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is up and z is towards you
    //     (i.e. the back of camera)
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is down and z is away from you (into the tag).
    // But we want:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is down and z is straight
    //     ahead
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is up and z is towards you (out of the tag).
    // Using these frames together with cv::solvePnP directly avoids
    // AprilTag 2's frames altogether.
    // TODO solvePnP[Ransac] better?
    std::vector<cv::Point3d> standaloneTagObjectPoints;
    std::vector<cv::Point2d> standaloneTagImagePoints;
    addObjectPoints(tag_size / 2, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(detection, standaloneTagImagePoints);
    auto const transform = getRelativeTransform(standaloneTagObjectPoints, standaloneTagImagePoints, fx, fy, cx, cy);
    Eigen::Quaternion<double> const rot_quaternion(Eigen::Matrix3d{transform.block(0, 0, 3, 3)});

    // Add the detection to the back of the tag detection array
    AprilTagDetection tag_detection;
    tag_detection.pose = makeTagPose(transform, rot_quaternion, image->header);
    tag_detection.id.push_back(detection->id);
    tag_detection.size.push_back(tag_size);
    tag_detection_array.detections.push_back(tag_detection);
    detection_names.push_back(standaloneDescription->frame_name());
  }

  //=================================================================
  // Estimate bundle origin pose for each bundle in which at least one
  // member tag was detected

  for (auto const& tag_bundle_description : tag_bundle_descriptions_) {
    // Get bundle name
    std::string const bundleName = tag_bundle_description.name();

    auto const it = bundleObjectPoints.find(bundleName);
    if (it != bundleObjectPoints.end()) {
      // Some member tags of this bundle were detected, get the bundle's position!

      auto const transform =
        getRelativeTransform(bundleObjectPoints[bundleName], bundleImagePoints[bundleName], fx, fy, cx, cy);
      Eigen::Quaterniond const rot_quaternion{Eigen::Matrix3d{transform.block(0, 0, 3, 3)}};

      // Add the detection to the back of the tag detection array
      AprilTagDetection tag_detection;
      tag_detection.pose = makeTagPose(transform, rot_quaternion, image->header);
      tag_detection.id   = tag_bundle_description.bundleIds();
      tag_detection.size = tag_bundle_description.bundleSizes();
      tag_detection_array.detections.push_back(tag_detection);
      detection_names.push_back(tag_bundle_description.name());
    }
  }

  // If set, publish the transform /tf topic
  if (publish_tf_) {
    for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose   = tag_detection_array.detections[i].pose.pose.pose;
      pose.header = tag_detection_array.detections[i].pose.header;
      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF(pose, tag_transform);
      tf_pub_.sendTransform(
        tf::StampedTransform(tag_transform, tag_transform.stamp_, camera_tf_frame_, detection_names[i]));
    }
  }

  return tag_detection_array;
}

int TagDetector::idComparison(const void* first, const void* second) {
  int const id1 = static_cast<apriltag_detection_t const*>(first)->id;
  int const id2 = static_cast<apriltag_detection_t const*>(second)->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates() {
  zarray_sort(detections_.get(), &idComparison);
  int count               = 0;
  bool duplicate_detected = false;
  while (true) {
    if (count > zarray_size(detections_.get()) - 1) {
      // The entire detection set was parsed
      return;
    }
    apriltag_detection_t* detection = nullptr;
    zarray_get(detections_.get(), count, &detection);
    int const id_current = detection->id;
    // Default id_next value of -1 ensures that if the last detection
    // is a duplicated tag ID, it will get removed
    int id_next = -1;
    if (count < zarray_size(detections_.get()) - 1) {
      zarray_get(detections_.get(), count + 1, &detection);
      id_next = detection->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected)) {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_.get(), count, shuffle);
      if (id_current != id_next) {
        ROS_WARN_STREAM("Pruning tag ID " << id_current
                                          << " because it "
                                             "appears more than once in the image.");
        duplicate_detected = false;  // Reset
      }
      continue;
    } else {
      count++;
    }
  }
}

void TagDetector::addObjectPoints(double const s, cv::Matx44d const& T_oi, std::vector<cv::Point3d>& objectPoints) {
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
  objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
  objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
  objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints(apriltag_detection_t* detection, std::vector<cv::Point2d>& imagePoints) {
  // Add to image point vector the tag corners in the image frame
  // Going counterclockwise starting from the bottom left corner
  std::array<double, 4> const tag_x{-1, 1, 1, -1};
  std::array<double, 4> const tag_y{1, 1, -1, -1};  // Negated because AprilTag tag local
                                                    // frame has y-axis pointing DOWN
                                                    // while we use the tag local frame
                                                    // with y-axis pointing UP
  for (std::size_t i = 0; i < 4; i++) {
    // Homography projection taking tag local frame coordinates to image pixels
    double im_x = 0.0;
    double im_y = 0.0;
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.emplace_back(im_x, im_y);
  }
}

Eigen::Matrix4d TagDetector::getRelativeTransform(std::vector<cv::Point3d> const& objectPoints,
                                                  std::vector<cv::Point2d> const& imagePoints, double const fx,
                                                  double const fy, double const cx, double const cy) {
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Matx33d const cameraMatrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Vec4f const distCoeffs(0, 0, 0, 0);  // distortion coefficients
  // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

  Eigen::Matrix4d T;  // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;
  return T;
}

geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(const Eigen::Matrix4d& transform,
                                                                  const Eigen::Quaternion<double>& rot_quaternion,
                                                                  const std_msgs::Header& header) {
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = header;
  //===== Position and orientation
  pose.pose.pose.position.x    = transform(0, 3);
  pose.pose.pose.position.y    = transform(1, 3);
  pose.pose.pose.position.z    = transform(2, 3);
  pose.pose.pose.orientation.x = rot_quaternion.x();
  pose.pose.pose.orientation.y = rot_quaternion.y();
  pose.pose.pose.orientation.z = rot_quaternion.z();
  pose.pose.pose.orientation.w = rot_quaternion.w();
  return pose;
}

void TagDetector::drawDetections(cv_bridge::CvImagePtr const& image) {
  for (int i = 0; i < zarray_size(detections_.get()); i++) {
    apriltag_detection_t* det = nullptr;
    zarray_get(detections_.get(), i, &det);

    // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    int const tagID        = det->id;
    bool is_part_of_bundle = false;
    for (auto const& bundle : tag_bundle_descriptions_) {
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) {
        is_part_of_bundle = true;
        break;
      }
    }
    // If not part of a bundle, check if defined as a standalone tag
    StandaloneTagDescription* standaloneDescription = nullptr;
    if (!is_part_of_bundle && !findStandaloneTagDescription(tagID, standaloneDescription, false)) {
      // Neither a standalone tag nor part of a bundle, so this is a "rogue"
      // tag, skip it.
      continue;
    }

    // Draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-left corner in
    // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
    // colors!
    line(image->image, cv::Point(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1])),
         cv::Point(static_cast<int>(det->p[1][0]), static_cast<int>(det->p[1][1])), cv::Scalar(0, 0xff, 0));  // green
    line(image->image, cv::Point(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1])),
         cv::Point(static_cast<int>(det->p[3][0]), static_cast<int>(det->p[3][1])), cv::Scalar(0, 0, 0xff));  // red
    line(image->image, cv::Point(static_cast<int>(det->p[1][0]), static_cast<int>(det->p[1][1])),
         cv::Point(static_cast<int>(det->p[2][0]), static_cast<int>(det->p[2][1])), cv::Scalar(0xff, 0, 0));  // blue
    line(image->image, cv::Point(static_cast<int>(det->p[2][0]), static_cast<int>(det->p[2][1])),
         cv::Point(static_cast<int>(det->p[3][0]), static_cast<int>(det->p[3][1])), cv::Scalar(0xff, 0, 0));  // blue

    // Print tag ID in the middle of the tag
    cv::String const text   = std::to_string(det->id);
    int const fontface      = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double const fontscale  = 0.5;
    int baseline            = 0;
    cv::Size const textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
    cv::putText(
      image->image, text,
      cv::Point(static_cast<int>(det->c[0] - textsize.width / 2), static_cast<int>(det->c[1] + textsize.height / 2)),
      fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags(XmlRpc::XmlRpcValue const& standalone_tags) {
  // Create map that will be filled by the function and returned in the end
  std::map<int, StandaloneTagDescription> descriptions;
  // Ensure the type is correct
  ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Loop through all tag descriptions
  for (int32_t i = 0; i < standalone_tags.size(); i++) {
    // i-th tag description
    XmlRpc::XmlRpcValue const& tag_description = standalone_tags[i];

    // Assert the tag description is a struct
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    // Assert type of field "id" is an int
    ROS_ASSERT(tag_description["id"s].getType() == XmlRpc::XmlRpcValue::TypeInt);
    // Assert type of field "size" is a double
    ROS_ASSERT(tag_description["size"s].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    auto const id = static_cast<int>(tag_description["id"s]);  // tag id
    // Tag size (square, side length in meters)
    auto const size = static_cast<double>(tag_description["size"s]);

    // Custom frame name, if such a field exists for this tag
    std::string frame_name;
    if (tag_description.hasMember("name"s)) {
      // Assert type of field "name" is a string
      ROS_ASSERT(tag_description["name"s].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = static_cast<std::string>(tag_description["name"s]);
    } else {
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_"s << id;
      frame_name = frame_name_stream.str();
    }

    StandaloneTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name);
    // Add this tag's description to map of descriptions
    descriptions.insert(std::make_pair(id, description));
  }

  return descriptions;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription> TagDetector::parseTagBundles(XmlRpc::XmlRpcValue const& tag_bundles) {
  std::vector<TagBundleDescription> descriptions;
  ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // Loop through all tag bundle descritions
  for (int32_t i = 0; i < tag_bundles.size(); ++i) {
    ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    // i-th tag bundle description
    XmlRpc::XmlRpcValue const& bundle_description = tag_bundles[i];

    std::string bundleName;
    if (bundle_description.hasMember("name"s)) {
      ROS_ASSERT(bundle_description["name"s].getType() == XmlRpc::XmlRpcValue::TypeString);
      bundleName = static_cast<std::string>(bundle_description["name"s]);
    } else {
      bundleName = "bundle_"s + std::to_string(i);
    }
    TagBundleDescription bundle_i(bundleName);
    ROS_INFO_STREAM("Loading tag bundle " << bundle_i.name());

    ROS_ASSERT(bundle_description["layout"s].getType() == XmlRpc::XmlRpcValue::TypeArray);
    XmlRpc::XmlRpcValue const& member_tags = bundle_description["layout"s];

    // Loop through each member tag of the bundle
    for (int32_t j = 0; j < member_tags.size(); ++j) {
      ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue const& tag = member_tags[j];

      ROS_ASSERT(tag["id"s].getType() == XmlRpc::XmlRpcValue::TypeInt);
      int const id = tag["id"s];

      ROS_ASSERT(tag["size"s].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      double const size = tag["size"s];

      // Make sure that if this tag was specified also as standalone,
      // then the sizes match
      StandaloneTagDescription* standaloneDescription = nullptr;
      if (findStandaloneTagDescription(id, standaloneDescription, false)) {
        ROS_ASSERT(size == standaloneDescription->size());
      }

      // Get this tag's pose with respect to the bundle origin
      double const x  = xmlRpcGetDoubleWithDefault(tag, "x"s, 0.);
      double const y  = xmlRpcGetDoubleWithDefault(tag, "y"s, 0.);
      double const z  = xmlRpcGetDoubleWithDefault(tag, "z"s, 0.);
      double const qw = xmlRpcGetDoubleWithDefault(tag, "qw"s, 1.);
      double const qx = xmlRpcGetDoubleWithDefault(tag, "qx"s, 0.);
      double const qy = xmlRpcGetDoubleWithDefault(tag, "qy"s, 0.);
      double const qz = xmlRpcGetDoubleWithDefault(tag, "qz"s, 0.);
      Eigen::Quaterniond q_tag(qw, qx, qy, qz);
      q_tag.normalize();
      Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

      // Build the rigid transform from tag_j to the bundle origin
      cv::Matx44d T_mj(R_oi(0, 0), R_oi(0, 1), R_oi(0, 2), x, R_oi(1, 0), R_oi(1, 1), R_oi(1, 2), y, R_oi(2, 0),
                       R_oi(2, 1), R_oi(2, 2), z, 0, 0, 0, 1);

      // Register the tag member
      bundle_i.addMemberTag(id, size, std::move(T_mj));
      ROS_INFO_STREAM(" " << j << ") id: " << id << ", size: " << size << ", "
                          << "p = [" << x << "," << y << "," << z << "], "
                          << "q = [" << qw << "," << qx << "," << qy << "," << qz << "]");
    }
    descriptions.push_back(bundle_i);
  }
  return descriptions;
}

double TagDetector::xmlRpcGetDouble(XmlRpc::XmlRpcValue const& xmlValue, std::string const& field) {
  ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
             (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));

  auto ret_val = xmlValue[field];
  return ret_val.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(ret_val) : static_cast<double>(ret_val);
}

double TagDetector::xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue const& xmlValue, std::string const& field,
                                               double const defaultValue) {
  if (xmlValue.hasMember(field)) {
    ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    auto ret_val = xmlValue[field];
    return ret_val.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(ret_val) : static_cast<double>(ret_val);
  }

  return defaultValue;
}

bool TagDetector::findStandaloneTagDescription(int const id, StandaloneTagDescription*& descriptionContainer,
                                               bool const printWarning) {
  auto const description_itr = standalone_tag_descriptions_.find(id);
  if (description_itr == standalone_tag_descriptions_.end()) {
    if (printWarning) {
      ROS_WARN_THROTTLE(10.0,
                        "Requested description of standalone tag ID [%d],"
                        " but no description was found...",
                        id);
    }
    return false;
  }
  descriptionContainer = &(description_itr->second);
  return true;
}

}  // namespace apriltag_ros
