#include "AlphaDet.h"
#include "alpha_detection.h"
#include "alpha_merge.h"
#include "image_list_gen.h"
#include <fstream>

using namespace hobot::vision::alpha;
namespace hb = hobot;

class AlphaDetImpl: public AlphaDet{
public:
    int init(std::vector <std::string> &model_names);
    int detect(int img_w, int img_h, int img_step, char *img, std::vector<std::vector<float>> &faces_results);

protected:
    std::vector <std::string> m_models;
    AlphaDetector *mp_det;
    ImageListGen *mp_roi_gen;
    GreyImagePyramidFP *mp_img_pyr;
    std::vector <std::list<SDetRespFP>> m_raw_resp_list;
    std::vector <std::list<SDetRespFP>> m_merged_resp_list;
};

int AlphaDetImpl::init(std::vector <std::string> &model_names)
{
    /// 1. Initiate Alpha detector
    const int max_img_w = 2000;
    const int max_img_h = 2000;
    mp_det = new AlphaDetector(max_img_w, max_img_h);

    /// 1.1 Load models
    m_models = model_names;
    // get a vector of istream to models
    std::vector < std::istream * > iss;
    for (unsigned int i = 0; i < m_models.size(); i++) {
        std::ifstream *ifs = new std::ifstream(m_models[i].c_str(), std::ifstream::binary);
        if (*ifs) {
            iss.push_back(ifs);
        } else {
            printf("Failed in loading model %s\n", m_models[i].c_str());
            return -1;
        }
    }
    // initialize these models
    mp_det->InitModels(iss);
    for (unsigned int i = 0; i < iss.size(); i++) {
        delete iss[i];
    }

    // get model reference size
    int model_num = mp_det->GetModelNum();
    for (int i = 0; i < model_num; i++) {
        int ref_w, ref_h;
        mp_det->GetModelRefSize(i, ref_w, ref_h);
        printf("model #%d: ref size %dx%d\n", i, ref_w, ref_h);
    }

    ///  Initiate image pyramid
    mp_img_pyr = new GreyImagePyramidFP(m_pad_border,
                                       m_scale_level_num,
                                       m_min_scale_level_w,
                                       m_min_scale_level_h,
                                       m_scale_bits,
                                       m_start_scale_denom,
                                       m_scale_step_denom,
                                       m_Nyquist_freq_ratio);

    // set roi
    mp_roi_gen = new GridImageListGen("roi");

    // initiate detection result containers
    m_raw_resp_list.resize(m_models.size());
    m_merged_resp_list.resize(m_raw_resp_list.size());
}

int AlphaDetImpl::detect(int img_w, int img_h, int img_step, char *img, std::vector<std::vector<float>> &faces_results)
{
    /// Compute image pyramid
    mp_img_pyr->Init(img_w, img_h, img_step, (unsigned char *) img);

    /// Scan for raw detection
    const ScanMode scan_mode = kCellSearch;
    const int coarse_to_fine_layer_num = 10;
    std::vector<const GreyImage *> img_list;
    std::vector<float> scale_factor;
    std::vector < std::vector < hobot::TSRect < int >> > image_roi_l;
    mp_roi_gen->GenerateImageList(*mp_img_pyr, img_list, scale_factor, image_roi_l);
    mp_det->Detect(img_list, scale_factor, image_roi_l, mp_img_pyr->GetPadBorder(),
               scan_mode, coarse_to_fine_layer_num, m_raw_resp_list);

    /// Merge and non-max suppression
    for (unsigned int ci = 0; ci < m_merged_resp_list.size(); ci++) {
        DetRespOnlineClusteringFP(m_raw_resp_list[ci], m_merge_overlap_ratio_thres,
                                  m_merged_resp_list[ci]);
        NonMaximumSuppresionFP(m_merged_resp_list[ci], m_nms_conf_thres,
                               m_nms_max_overlap_ratio, m_nms_max_contain_ratio);
    }

    /// Get detected result
    faces_results.clear();
    for (unsigned int ci = 0; ci < m_merged_resp_list.size(); ci++) {
        std::vector<float> faces_result;
        printf("Model #%u: %lu raw, %lu merged. ", ci, m_raw_resp_list[ci].size(),
               m_merged_resp_list[ci].size());
        for (std::list<SDetRespFP>::iterator itr = m_merged_resp_list[ci].begin();
             itr != m_merged_resp_list[ci].end(); itr++) {
            float left = float(itr->rect.l) / (1 << kCoordDecPrec);
            float top = float(itr->rect.t) / (1 << kCoordDecPrec);
            float right = float(itr->rect.r) / (1 << kCoordDecPrec);
            float bottom = float(itr->rect.b) / (1 << kCoordDecPrec);
            float conf = float(itr->conf) / (1 << kScoreDecPrec);

            faces_result.push_back(left);
            faces_result.push_back(top);
            faces_result.push_back(right);
            faces_result.push_back(bottom);
            faces_result.push_back(conf);
        }

        printf("\n");
        faces_results.push_back(faces_result);
    }
}


AlphaDet::~AlphaDet(){
    if(mp_alphaDetImpl)
        delete mp_alphaDetImpl;
}

int AlphaDet::init(std::vector <std::string> &model_names)
{
    if(mp_alphaDetImpl == NULL)
        mp_alphaDetImpl = new AlphaDetImpl();

    return mp_alphaDetImpl->init(model_names);
}

int AlphaDet::detect(int img_w, int img_h, int img_step, char *img, std::vector<std::vector<float>> &faces_results)
{
    return mp_alphaDetImpl->detect(img_w, img_h, img_step, img, faces_results);
}
