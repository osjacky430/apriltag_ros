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
 *
 ** common_functions.h *********************************************************
 *
 * Wrapper classes for AprilTag standalone and bundle detection. Main function
 * is TagDetector::detectTags which wraps the call to core AprilTag 2
 * algorithm, apriltag_detector_detect().
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:23:14 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_APRILTAG_WRAPPER_HPP_
#define APRILTAG_ROS_APRILTAG_WRAPPER_HPP_

#include <apriltag.h>
#include <memory>
#include <string>

#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

namespace apriltag_ros {

struct ApriltagDetectorDeleter {
  void operator()(apriltag_detector_t* t_detector) const noexcept { apriltag_detector_destroy(t_detector); }
};

static inline auto get_apriltag_family_creator_deleter(std::string const& t_family_name) {
  using namespace std::string_literals;

  if (t_family_name == "tagStandard52h13"s) {
    return std::make_pair(tagStandard52h13_create, tagStandard52h13_destroy);
  } else if (t_family_name == "tagStandard41h12"s) {
    return std::make_pair(tagStandard41h12_create, tagStandard41h12_destroy);
  } else if (t_family_name == "tag36h11"s) {
    return std::make_pair(tag36h11_create, tag36h11_destroy);
  } else if (t_family_name == "tag25h9"s) {
    return std::make_pair(tag25h9_create, tag25h9_destroy);
  } else if (t_family_name == "tag16h5"s) {
    return std::make_pair(tag16h5_create, tag16h5_destroy);
  } else if (t_family_name == "tagCustom48h12"s) {
    return std::make_pair(tagCustom48h12_create, tagCustom48h12_destroy);
  } else if (t_family_name == "tagCircle21h7"s) {
    return std::make_pair(tagCircle21h7_create, tagCircle21h7_destroy);
  } else if (t_family_name == "tagCircle49h12"s) {
    return std::make_pair(tagCircle49h12_create, tagCircle49h12_destroy);
  }

  throw std::invalid_argument("Invalid tag family specified");
}

struct ApriltagDetectorParam {
  float quad_decimate_ = 1.0;
  float quad_sigma_    = 0.0;
  int thread_count_    = 4;
  int debug_           = 0;
  int refine_edges_    = 1;
};

static inline auto make_apriltag_family(std::string const& t_family_name) {
  auto const creator_deleter = get_apriltag_family_creator_deleter(t_family_name);

  auto const creator = creator_deleter.first;
  auto const deleter = creator_deleter.second;
  std::shared_ptr<apriltag_family_t> ret_val(creator(), deleter);

  return ret_val;
}

static inline auto make_apriltag_detector(ApriltagDetectorParam const& t_param = ApriltagDetectorParam{}) noexcept {
  std::shared_ptr<apriltag_detector_t> detector_(apriltag_detector_create(), ApriltagDetectorDeleter{});
  detector_->quad_decimate = t_param.quad_decimate_;
  detector_->quad_sigma    = t_param.quad_sigma_;
  detector_->nthreads      = t_param.thread_count_;
  detector_->debug         = t_param.debug_;
  detector_->refine_edges  = t_param.refine_edges_;

  return detector_;
}

struct ApriltagDetectionArrayDeleter {
  void operator()(zarray_t* t_detection_array) const noexcept { apriltag_detections_destroy(t_detection_array); }
};

static inline auto apriltag_detection_wrapper_fn(apriltag_detector_t* const t_detector, image_u8_t* const t_image) {
  return std::shared_ptr<zarray_t>(apriltag_detector_detect(t_detector, t_image), ApriltagDetectionArrayDeleter{});
}

}  // namespace apriltag_ros

#endif