//
//  Merge Responses for Alpha Detection
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#include "alpha_scan.h"
#include <list>

namespace hobot {
namespace vision {
namespace alpha {

// decimal precision for division operator during weighted average
const int kDivDecPrec = 12;
// decimal precision for conf_thres, max_overlap and max_contain
const int kMergeRatioDecPrec = 10;

int DetRespOnlineClusteringFP(const std::list<SDetRespFP> &raw_resp_list,     // input detection responses
                              const float overlap_ratio_thres,                                // threshold for merge two resposes
                              std::list<SDetRespFP> &merged_resp_list);                       // merged results

int NonMaximumSuppresionFP(std::list<SDetRespFP> &resp_list,    // input detection responses
                           const float conf_thres,                               // confidence threshold
                           const float max_overlap,                              // max overlap ratio
                           const float max_contain);                             // max contain ratio

} // namespace alpha
} // namespace vision
} // namespace hobot
