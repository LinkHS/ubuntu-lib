//
//  Scanning for Alpha Detection
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#ifndef __ALPHA_SCANNING_H__
#define __ALPHA_SCANNING_H__

#include <list>
#include <vector>
#include "base.h"

namespace hobot {
namespace vision {
namespace alpha {

struct AlphaCascade;
//struct ImageMcms;
class ImageMcms;

//
// Scanning modes
//
enum ScanMode {
  kUniformScan = 0,
  kCoarse2FineScan,
  kCellSearch
};

//
//  Detection response (fix-point version)
//
struct SDetRespFP {
  TSRect<int> rect;

  int conf;

  SDetRespFP() {
    conf = 0;
  }
};

#ifdef CAL_DET_STAT
//
//  Detection statistics
//
struct DetStat {
  int64 image_pixel_num;
  int64 scanned_wnd_num;
  int64 layer_num;
  int64 wnd_num_each_layer[64];
  int64 cum_dt_num_each_layer[64];

  DetStat() { Reset(); }

  void Reset() {
    image_pixel_num = scanned_wnd_num = 0;
    layer_num = 0;
    for (int i = 0; i < 64; i++) {
      wnd_num_each_layer[i] = 0;
      cum_dt_num_each_layer[i] = 0;
    }
  }
};
#endif

// fixed point decimal precision for coordinate
const int kCoordDecPrec = 2;
// fixed point score precision
const int kScoreDecPrec = 8;

//
//  Scan a Mcms image with one cascade
//
int AlphaScanFP(AlphaCascade &cascade,
                const unsigned char *feat_ptr,
                const int ystep, const int xstep,
                const int bgn_x, const int bgn_y,
                const int end_x, const int end_y,
                const ScanMode scan_mode,
                const int coarse2fine_layer_num,
                const int coord_scale_numer, const int coord_scale_bits,
                std::list<SDetRespFP> &raw_resp_list,
                const int bias_x = 0, const int bias_y = 0
#ifdef CAL_DET_STAT
    , DetStat &det_stat
#endif
);

} // namespace alpha
} // namespace vision
} // namespace hobot

#endif // __SCANNING_H__
