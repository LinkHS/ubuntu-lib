//
// Created by austin on 7/5/17.
//

#ifndef PROJECT_ALPHADET_H
#define PROJECT_ALPHADET_H

#include <vector>
#include <string>

class AlphaDetImpl;

class AlphaDet{
public:
    ~AlphaDet();

    int init(std::vector <std::string> &model_names);
    int detect(int img_w, int img_h, int img_step, char *img, std::vector<std::vector<float>> &faces_results);

protected:
    /// image pyramid parameters
    int m_pad_border = 16;
    int m_scale_level_num = 12;
    int m_min_scale_level_w = 64;
    int m_min_scale_level_h = 64;
    int m_scale_bits = 4;
    int m_start_scale_denom = 16;
    int m_scale_step_denom = 20;
    float m_Nyquist_freq_ratio = 1.0f;

    /// Merge and non-max suppression
    float m_merge_overlap_ratio_thres = 0.5f;
    float m_nms_max_overlap_ratio = 0.6f;
    float m_nms_max_contain_ratio = 0.8f;
    float m_nms_conf_thres = 4.0f;
private:
    AlphaDetImpl *mp_alphaDetImpl = NULL;
};

#endif //PROJECT_ALPHADET_H
