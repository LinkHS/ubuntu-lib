//
// Created by Bear on 2017/6/18.
// Copyright (c) 2017 - 2022. Horizon Robotics. All rights reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.
//

#ifndef ALPHA_DET_PREDICTION_IMAGE_LIST_GEN_H
#define ALPHA_DET_PREDICTION_IMAGE_LIST_GEN_H

#include <string>

#include "base.h"
#include "grey_image_pyramid.h"

namespace hobot
{
namespace vision
{
namespace alpha
{

class ImageListGen {
 public:
  explicit ImageListGen(std::string inst = "", std::string cls = "")
      : instance_name_(inst), class_name_(cls)
  {
  }

  virtual void Reset() = 0;

  virtual void Config(std::string config = "") = 0;

  virtual void GenerateImageList(const GreyImagePyramid &img_pyr,
                                 std::vector<const GreyImage *> &img_list,
                                 std::vector<float> &scale_factor,
                                 std::vector<std::vector<TSRect<int> > > &image_roi_l) = 0;

  std::string GetInstanceName() const
  {
    return instance_name_;
  }

  std::string GetClassName() const
  {
    return class_name_;
  }

 private:
  std::string instance_name_;

  std::string class_name_;
};

class GridImageListGen : public ImageListGen {
 public:
  explicit GridImageListGen(std::string inst = "")
      : ImageListGen(inst, "GridImageListGen"), round_(0)
  {
  }

  void Reset() override;

  void Config(std::string config = "") override;

  void GenerateImageList(const GreyImagePyramid &img_pyr,
                         std::vector<const GreyImage *> &img_list,
                         std::vector<float> &scale_factor,
                         std::vector<std::vector<TSRect<int> > > &image_roi_l) override;

 private:
  std::vector<float> grid_x_;
  std::vector<float> grid_y_;
  int round_;
};

} //  namespace alpha
} //  namespace vision
} //  namespace hobot

#endif //ALPHA_DET_PREDICTION_IMAGE_LIST_GEN_H
