//
//  Resizing Image Functions and Classes
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#ifndef __IMAGE_RESIZER_H__
#define __IMAGE_RESIZER_H__

#include "base.h"
#include <vector>

namespace hobot {
namespace vision {
namespace alpha {

//
// Resize grey image by bilinear interpolation.
// Source and destination size as well as step are given.
// buffer is needed. set buf as NULL to use native dynamic buffer.
// set src_img or dst_img to be NULL so as to get needed buffer size.
//
int ResizeBilinearInterpolation(const int src_w, const int src_h,
                                const int src_step, const uchar *src_img,
                                const int dst_w, const int dst_h,
                                const int dst_step, uchar *dst_img,
                                const int buf_size = 0, uint *buf = NULL);

//
// Resize grey image by bilinear interpolation (hardware friendly version).
// h and v for horizontal and vertical respectively
// d and b for scale denominator and numeriator (in bit) respectively
// horizontal scale = 2^hb / hd
// vertical scale = 2^vb / vd
//
int ResizeBilinearInterpolationHW(const int src_w, const int src_h,
                                  const int src_step, const uchar *src_img,
                                  const int hb, const int hd,
                                  const int vb, const int vd,
                                  const int dst_step, uchar *dst_img,
                                  int &dst_w, int &dst_h,
                                  const int buf_size = 0, uint *buf = NULL);

void GetFilter5ForDownSample(const float down_sample_scale,
                             const float Nyquist_freq_ratio, uchar *filter5);

int ResizeBilinearInterpolationGC(const int src_w, const int src_h,
                                  const int src_step, const uchar *src_img,
                                  const int hb, const int hd,
                                  const int vb, const int vd,
                                  int &dst_w, int &dst_h,
                                  const int dst_step, uchar *dst_img);

//
//  Class for resizing grey images with interior buffer (Thread NOT safe)
//
class GreyImageResizer {
 protected:
  size_t shrink_buf_size_;

  uchar *shrink_buf_;

  size_t buf_size_;

  uint *buf_;

  size_t filter_buf_size_;

  uchar *filter_buf_[2];

 public:
  GreyImageResizer();
  virtual ~GreyImageResizer();

  void ResizeGreyImage(const int src_w, const int src_h,
                       const int src_step, const uchar *src_img,
                       const int dst_w, const int dst_h,
                       const int dst_step, uchar *dst_img);

  // resize a grey image (hardware friendly version)
  // only calculate dst_w and dst_h if src_img == NULL
  void ResizeGreyImageHW(const int src_w, const int src_h,
                         const int src_step, const uchar *src_img,
                         const int hor_b, const int hor_d,
                         const int hor_filter_sz, const uchar *hor_filter,
                         const int ver_b, const int ver_d,
                         const int ver_filter_sz, const uchar *ver_filter,
                         const int dst_step, uchar *dst_img,
                         int &dst_w, int &dst_h);

  //
  //  Grey image resizer horizontally filtered
  //  Gaussian filter is adopted to alliviate artifact during resizing, balancing aliasing
  //  effect and bluriness problem, while optimized for further hardware implementation.
  //
  //  Gaussian filter kernel size satisfies /sigma^2 = -8 * f^2 * \ln\lambda, where f is
  //  the sampling rate, i.e., N = n * f, N is the original size, while n is the new size
  //  after resizing. 0 < \lambda < 1 is the Nyquist frequency ratio you want to preserve.
  //  Vertically, simple bilinear interpolation is adopted as line buffers are expensive!
  //
  void ResizeGreyImageHorFilter5(const int src_w, const int src_h,
                                 const int src_step, const uchar *src_img,
                                 const int hor_b, const int hor_d,
                                 const int ver_b, const int ver_d,
                                 const float Nyquist_freq_ratio,
                                 const int dst_step, uchar *dst_img,
                                 int &dst_w, int &dst_h);
};

} // namespace alpha
} // namespace vision
} // namespace hobot

#endif //__IMAGE_RESIZER_H__
