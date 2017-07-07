//
//  Alpha Detection
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#include "grey_image_pyramid.h"
#include "alpha_scan.h"
#include <list>
#include <vector>

namespace hobot {
namespace vision {
namespace alpha {

struct AlphaCascade;
//struct ImageMcms;
class ImageMcms;

const int kImageScaleRecipDecPrec = 6;

//
//  Detect an image at multiple scales with multiple detectors
//
struct AlphaDetector {
  std::vector<AlphaCascade *> cascades_;            //cascade detection models
  ImageMcms *mcms_;                                 //multi-channel multi-scale feature
  bool as_golden_c_;

  AlphaDetector(int max_img_w, int max_img_h, bool as_golden_c=false);

  virtual ~AlphaDetector();

  void Release();

  void InitModels(std::vector<std::istream *> &iss);

  int GetModelNum() const;

  bool GetModelRefSize(const int model_i, int &ref_w, int &ref_h) const;

  void Detect(const GreyImagePyramid &img_pyr,
              ScanMode scan_mode, int coarse2fine_layer_num,
              std::vector<std::list<SDetRespFP> > &raw_resp_list);

  void Detect(const GreyImagePyramid &img_pyr,
              ScanMode scan_mode, int coarse2fine_layer_num,
              std::vector<TSRect<int> > image_roi_l,
              std::vector<std::list<SDetRespFP> > &raw_resp_list);
  void Detect(std::vector<const GreyImage *> &img_list,
              const std::vector<float> &scale_factor,
              const std::vector<std::vector<TSRect<int> > > &image_roi_l,
              const int pad_border,
              ScanMode scan_mode, int coarse2fine_layer_num,
              std::vector<std::list<SDetRespFP> > &raw_resp_list);

#ifdef CAL_DET_STAT
  std::vector<DetStat>        model_stats_;         //detection statistics of each model
  void ResetDetStat();                              //reset statistics
  void PrintDetStat();                              //print statistics
#endif

#ifdef SHOW_TIME
  long long GetTimeStamp();
  void ClearTimer();
  long long tmp_timer_;
  long long mcms_time_;
  long long scan_time_;
#endif
};

} // namespace alpha
} // namespace vision
} // namespace hobot
