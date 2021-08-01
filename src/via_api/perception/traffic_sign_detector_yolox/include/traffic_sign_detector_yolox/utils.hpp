// This file is wirtten base on the following file:
// https://github.com/Tencent/ncnn/blob/master/examples/yolov5.cpp
// Copyright (C) 2020 THL A29 Limited, a Tencent company. All rights reserved.
// Licensed under the BSD 3-Clause License (the "License"); you may not use this
// file except in compliance with the License. You may obtain a copy of the
// License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
// ------------------------------------------------------------------------------
// Copyright (C) 2020-2021, Megvii Inc. All rights reserved.

#ifndef YOLOX_UTILS_HPP_
#define YOLOX_UTILS_HPP_

#include "layer.h"
#include "net.h"

#if defined(USE_NCNN_SIMPLEOCV)
#include "simpleocv.h"
#else
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif
#include <float.h>
#include <stdio.h>
#include <vector>

namespace via {
namespace perception {
namespace traffic_sign {

// YOLOX use the same focus in yolov5
class YoloV5Focus : public ncnn::Layer {
 public:
  YoloV5Focus() { one_blob_only = true; }
  virtual int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob,
                      const ncnn::Option& opt) const {
    int w = bottom_blob.w;
    int h = bottom_blob.h;
    int channels = bottom_blob.c;

    int outw = w / 2;
    int outh = h / 2;
    int outc = channels * 4;

    top_blob.create(outw, outh, outc, 4u, 1, opt.blob_allocator);
    if (top_blob.empty()) return -100;

#pragma omp parallel for num_threads(opt.num_threads)
    for (int p = 0; p < outc; p++) {
      const float* ptr =
          bottom_blob.channel(p % channels).row((p / channels) % 2) +
          ((p / channels) / 2);
      float* outptr = top_blob.channel(p);

      for (int i = 0; i < outh; i++) {
        for (int j = 0; j < outw; j++) {
          *outptr = *ptr;

          outptr += 1;
          ptr += 2;
        }

        ptr += w;
      }
    }

    return 0;
  }
};

struct Object {
  cv::Rect_<float> rect;
  int label;
  float prob;
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

float intersection_area(const Object& a, const Object& b);

void qsort_descent_inplace(std::vector<Object>& faceobjects, int left,
                           int right);
void qsort_descent_inplace(std::vector<Object>& objects);

void nms_sorted_bboxes(const std::vector<Object>& faceobjects,
                       std::vector<int>& picked, float nms_threshold);

void generate_grids_and_stride(const int target_size, std::vector<int>& strides,
                               std::vector<GridAndStride>& grid_strides);

void generate_yolox_proposals(std::vector<GridAndStride> grid_strides,
                              const ncnn::Mat& feat_blob, float prob_threshold,
                              std::vector<Object>& objects);

void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);
int detect_yolox(ncnn::Net* yolox, const cv::Mat& bgr,
                 std::vector<Object>& objects);

}  // namespace traffic_sign
}  // namespace perception
}  // namespace via

#endif