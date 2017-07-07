//
//  Sample Codes of Using Alpha Detection Module
//

//#include <vld.h>
#include "alpha_detection.h"
#include "alpha_merge.h"
#include "image_list_gen.h"
#include <stdio.h>
#include <string>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"


#if defined(__linux__) || defined(__APPLE__)
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#define sprintf_s sprintf
#endif
#ifdef _WIN32
#include <windows.h>
#include <time.h>
#endif

using namespace hobot::vision::alpha;
namespace hb = hobot;

long long GetTimeStamp() {
#ifdef _WIN32
  LARGE_INTEGER freq, curr_time;
  QueryPerformanceFrequency(&freq);
  QueryPerformanceCounter(&curr_time);
  return curr_time.QuadPart * 1000 / freq.QuadPart;

#elif defined(__linux__) || defined(__APPLE__)
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  return curr_time.tv_sec * 1000 + curr_time.tv_usec / 1000;

#else
  REPORT_ERROR_POSITION
  ;
#endif
}


int main(int argc, char **argv) {
  //
  //  Initiate Alpha detector
  //
  const int max_img_w = 2048;
  const int max_img_h = 2048;
  AlphaDetector det(max_img_w, max_img_h);

  //
  //  Load models
  //
  std::vector<std::string> model_names = {
//    "../models/face/20160802_40x40_new_smp_tfn0015.lv19.20160801_40x40_release_version_new_dp.fp4.dp10.bin",
    "../models/face/20170426_40x40_height_28_p15_r4_i4_24_0128_R0128_F0050.ln24.fp8_10_4.bin"
  };
  // get a vector of istream to models
  std::vector<std::istream *> iss;
  for (unsigned int i = 0; i < model_names.size(); i++) {
    std::ifstream *ifs = new std::ifstream(model_names[i].c_str(), std::ifstream::binary);
    if (*ifs) {
      iss.push_back(ifs);
    }
    else {
      printf("Failed in loading model %s\n", model_names[i].c_str());
      return -1;
    }
  }
  // initialize these models
  det.InitModels(iss);
  for (unsigned int i = 0; i < iss.size(); i++) {
    delete iss[i];
  }

  // get model reference size
  int model_num = det.GetModelNum();
  for (int i = 0; i < model_num; i++) {
    int ref_w, ref_h;
    det.GetModelRefSize(i, ref_w, ref_h);
    printf("model #%d: ref size %dx%d\n", i, ref_w, ref_h);
  }

  //
  //  Initiate image pyramid
  //
  const int pad_border = 16;
  const int scale_level_num = 12;
  const int min_scale_level_w = 64;
  const int min_scale_level_h = 64;
  const int scale_bits = 4;
  const int start_scale_denom = 16;
  const int scale_step_denom = 20;
  const float Nyquist_freq_ratio = 1.0f;

  GreyImagePyramidFP
      img_pyr(pad_border,
              scale_level_num,
              min_scale_level_w,
              min_scale_level_h,
              scale_bits,
              start_scale_denom,
              scale_step_denom,
              Nyquist_freq_ratio);

  // set roi
  ImageListGen *roi_gen = new GridImageListGen("roi");

  //
  //  Load a raw image
  //
  //const int img_w = 512;
  //const int img_h = 512;
  //const int img_step = 512;
  int img_w = 512;
  int img_h = 512;
  int img_step = 512;
  std::string image_name = "../data/Lenna_512x512x1.raw";
  if (argc >= 2)
    image_name = argv[1];
  if (argc >= 3) {
    img_w = atoi(argv[2]);
    img_step = img_w;
  }
  if (argc >= 4)
    img_h = atoi(argv[3]);
  char *img = NULL;
  std::ifstream ifs(image_name.c_str(), std::ifstream::binary);
  if (ifs) {
    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    if (length != img_w * img_h) {
      printf("incorrect file length!\n");
      return -1;
    }
    img = new char[length];
    ifs.read(img, length);
  }
  else {
    printf("failed in loading raw image!\n");
    return -1;
  }

  //
  //  Detect the image
  //
  long long st = GetTimeStamp();
  std::vector<std::list<SDetRespFP> > raw_resp_list, merged_resp_list;
  for (int i = 0; i < 36; i++) {
    // initiate detection result containers
    raw_resp_list.resize(model_names.size());
    // compute image pyramid
    img_pyr.Init(img_w, img_h, img_step, (unsigned char*)img);
    // scan for raw detection
//    const ScanMode scan_mode = kCoarse2FineScan;
    const ScanMode scan_mode = kCellSearch;
    const int coarse_to_fine_layer_num = 10;
    std::vector<const GreyImage *> img_list;
    std::vector<float> scale_factor;
    std::vector<std::vector<hobot::TSRect<int>>> image_roi_l;
    roi_gen->GenerateImageList(img_pyr, img_list, scale_factor, image_roi_l);
    det.Detect(img_list, scale_factor, image_roi_l, img_pyr.GetPadBorder(),
               scan_mode, coarse_to_fine_layer_num, raw_resp_list);
//    det.Detect(img_pyr, scan_mode, coarse_to_fine_layer_num, raw_resp_list);
    // merge and non-max suppression
    const float merge_overlap_ratio_thres = 0.5f;
    const float nms_max_overlap_ratio = 0.6f;
    const float nms_max_contain_ratio = 0.8f;
    const float nms_conf_thres = 4.0f;
    merged_resp_list.resize(raw_resp_list.size());
    for (unsigned int ci = 0; ci < merged_resp_list.size(); ci++) {
      DetRespOnlineClusteringFP(raw_resp_list[ci], merge_overlap_ratio_thres,
                                merged_resp_list[ci]);
      NonMaximumSuppresionFP(merged_resp_list[ci], nms_conf_thres,
                             nms_max_overlap_ratio, nms_max_contain_ratio);
    }
  }
  long long time_elaps = GetTimeStamp() - st;

  //
  //  Print the results
  //
  for (unsigned int ci = 0; ci < merged_resp_list.size(); ci++) {
    printf("Model #%u: %lu raw, %lu merged. ", ci, raw_resp_list[ci].size(),
            merged_resp_list[ci].size());
    for (std::list<SDetRespFP>::iterator itr = merged_resp_list[ci].begin();
        itr != merged_resp_list[ci].end(); itr++) {
      float left = float(itr->rect.l) / (1 << kCoordDecPrec);
      float top = float(itr->rect.t) / (1 << kCoordDecPrec);
      float right = float(itr->rect.r) / (1 << kCoordDecPrec);
      float bottom = float(itr->rect.b) / (1 << kCoordDecPrec);
      float conf = float(itr->conf) / (1 << kScoreDecPrec);
      printf(" %.1f %.1f %.1f %.1f %.2f\n", left, top, right, bottom, conf);

      cv::Mat mRaw(img_h, img_w, CV_8UC1, img);
      cv::rectangle(mRaw, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255,0,0));
      cv::imshow("test.jpg", mRaw);
      cv::waitKey(0);
    }
    printf("\n");
  }
  printf("time spent %llu ms\n", time_elaps);

  delete []img;
  return 0;

}
