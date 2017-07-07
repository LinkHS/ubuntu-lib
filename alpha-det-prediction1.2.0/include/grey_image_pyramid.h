//
//  Classes for Building a Grey Image Pyramid
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#ifndef __GREY_IMAGE_PYRAMID_H__
#define __GREY_IMAGE_PYRAMID_H__

#include "base.h"
#include "image_resizer.h"
#include <vector>

namespace hobot {
namespace vision {
namespace alpha {

//
//  Gray Image Container
//
class GreyImage {
 protected:
  int width_, height_, step_;   //width, height and width-step
  uchar *data_;                 //pointer to image data
  size_t data_size_;            //actual data size
 public:
  GreyImage();
  GreyImage(const int width, const int height, const int step = -1);
  ~GreyImage();
  bool Reshape(const int width, const int height, const int step = -1);
  inline const int GetWidth() const { return width_; }
  inline const int GetHeight() const { return height_; }
  inline const int GetWidthStep() const { return step_; }
  inline uchar *GetData() const { return data_; }
  inline const uchar *GetConstData() const { return data_; }
};

//
//  Base Class for Building a Image Pyramid
//
class GreyImagePyramid {
 protected:
  std::vector<GreyImage *> image_l_;           //images at different levels
  int pad_border_;        //padding size at border of images
  std::vector<float> image_scale_l_;     //image scales
  int level_num_;         //pyramid level number
  int max_level_num_;     //maximum number of scale levels
  int min_level_w_;       //minimum level image width
  int min_level_h_;       //minimum level image height

 public:
  GreyImagePyramid(const int pad_border = 0,                         //pad border
                   const int max_level_num = -1,                     //maximum number of scale levels
                   const int min_level_w = -1,                       //minimum level image width
                   const int min_level_h = -1);                      //minimum level image height

  virtual ~GreyImagePyramid();

  virtual void Init(const int img_w,
                    const int img_h,                     //source image size
                    const int grey_img_step,
                    const uchar *img_data) = 0;  //step and image data

  //resize level image
  GreyImage *GetResizedLevelImage(const int lv_i,         //level index
                                  const int dst_w,        //dst width (without padding)
                                  const int dst_h);       //dst height (widthout padding)

  //get valid level number
  inline uint GetLevelNum() const { return level_num_; }

  //get padding border
  inline int GetPadBorder() const { return pad_border_; }

  //get level scale
  inline float GetScale(const int k) const {
    ERROR_IF(k < 0 || k >= level_num_,
             "k exceeds valid range, %d %d",
             k,
             level_num_);
    return image_scale_l_[k];
  }

  inline std::vector<float> GetScales() const {
    return image_scale_l_;
  }

  //get level image
  inline const GreyImage *GetLevel(const int k) const {
    ERROR_IF(k < 0 || k >= level_num_,
             "k exceeds valid range, %d %d",
             k,
             level_num_);
    return image_l_[k];
  }
};

//
//  Image Pyramid (Fixed Point Scale Mode)
//
class GreyImagePyramidFP : public GreyImagePyramid {
 protected:
  GreyImageResizer image_resizer_;       //an instance for resizing image
  int scale_numer_bit_;     //scale nemerator (in bit)
  int start_scale_denom_;   //denominator of starting scale
  int scale_step_denom_;    //denominator of scale step
  float Nyquist_freq_ratio_;  //Nyquist freq ratio you want to preserve
 public:
  GreyImagePyramidFP(const int pad_border = 0,                         //pad border
                     const int max_level_num = -1,                     //maximum number of scale levels
                     const int min_level_w = -1,                       //minimum level image width
                     const int min_level_h = -1,                       //minimum level image height
                     const int scale_numer_bit = 4,                    //scale nemerator (in bit)
                     const int start_scale_denom = 16,                 //denominator of starting scale
                     const int scale_step_denom = 20,                  //denominator of scale step
                     const float Nyquist_freq_ratio = 1.0f);           //Nyquist freq ratio you want to preserve

  virtual ~GreyImagePyramidFP();
  virtual void Init(const int img_w,
                    const int img_h,                  //source image size
                    const int grey_img_step,
                    const uchar *img_data);   //step and image data

  virtual void Init(const int img_w,
    const int img_h,                  //source image size
    const int grey_img_step,
    const uchar *img_data,
    std::vector<TSRect<int> > & image_roi_l,
    std::vector<float> & image_scale_l);   //step and image data  

};

//
//  Image Pyramid (Octave Mode)
//

class GreyImagePyramidOctave : public GreyImagePyramid {
 protected:
  GreyImageResizer image_resizer_;       //an instance for resizing image
  int scale_numer_bit_;     //scale nemerator (in bit)
  int scale_step_denom_;    //denominator of scale step
  int octave_num_;          //the number of octaves
  int sub_level_num_;       //the number of sub scale levels in an octave

 public:
  GreyImagePyramidOctave(const int pad_border = 0,                         //pad border
                         const int max_level_num = -1,                     //maximum number of scale levels
                         const int min_level_w = -1,                       //minimum level image width
                         const int min_level_h = -1,                       //minimum level image height
                         const int scale_numer_bit = 4,                    //scale nemerator (in bit)
                         const int scale_step_denom = 20,                  //denominator of scale step
                         const int octave_num = 4,                         //the number of octaves
                         const int sub_level_num = 3);                     //the number of sub scale levels in an octave

  virtual ~GreyImagePyramidOctave();

  virtual void Init(const int img_w,
                    const int img_h,                  //source image size
                    const int grey_img_step,
                    const uchar *img_data);   //step and image data
};

//
//  Image Pyramid (Golden C version)
//
class GreyImagePyramidGC : public GreyImagePyramid {
 protected:
  int scale_numer_bit_;     //scale nemerator (in bit)
  int scale_step_denom_;    //denominator of scale step

 public:
  GreyImagePyramidGC(const int pad_border = 0,                         //pad border
                     const int max_level_num = -1,                     //maximum number of scale levels
                     const int min_level_w = -1,                       //minimum level image width
                     const int min_level_h = -1,                       //minimum level image height
                     const int scale_numer_bit = 4,                    //scale nemerator (in bit)
                     const int scale_step_denom = 20);                 //denominator of scale step

  virtual ~GreyImagePyramidGC();

  virtual void Init(const int img_w,
                    const int img_h,                  //source image size
                    const int grey_img_step,
                    const uchar *img_data);   //step and image data
};



/*

  bool Init(const int img_w, const int img_h,                 //source image size
            const int grey_img_step, const uchar *img_data,   //step and image data
            const int pad_border,                             //pad border
            const int scale_numer_bit,                        //scale nemerator (in bit)
            const int start_scale_denom,                      //denominator of starting scale
            const int scale_step_denom,                       //denominator of scale step
            const int level_num,                              //the number of scale levels
            const float Nyquist_freq_ratio = 1.0f);           //Nyquist freq ratio you want to preserve


  bool Init(const int img_w, const int img_h,                 //source image size
            const int grey_img_step, const uchar *img_data,   //step and image data
            const int pad_border,                             //pad border
            const std::vector<TSize<int> > &dst_img_size_l);  //image size at different level

  bool Init(const int img_w, const int img_h,                 //source image size
            const int grey_img_step, const uchar *img_data,   //step and image data
            const int pad_border,                             //pad border
            const int scale_numer_bit,                        //scale nemerator (in bit)
            const int start_scale_denom,                      //denominator of starting scale
            const int scale_step_denom,                       //denominator of scale step
            const int level_num,                              //the number of scale levels
            const float Nyquist_freq_ratio = 1.0f);           //Nyquist freq ratio you want to preserve

  bool InitOctave(const int img_w, const int img_h,               //source image size
                const int grey_img_step, const uchar *img_data,   //step and image data
                const int pad_border,                             //pad border
                const int scale_numer_bit,                        //scale nemerator (in bit)
                const int scale_step_denom,                       //denominator of scale step
                const int sub_level_num,                          //the number of sub scale levels in an octave
                const int octave_num);                            //the number of octaves

  inline uint GetLevelNum() const { return image_l_.size(); }
  inline int GetPadBorder() const { return pad_border_; }
  float GetScale(const int k) const;
  const GreyImage* GetLevel(const int k) const;
};
*/

} // namespace alpha
} // namespace vision
} // namespace hobot

#endif //__GREY_IMAGE_PYRAMID_H__

