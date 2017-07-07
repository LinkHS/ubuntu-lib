#include "AlphaDet.h"
#include "opencv2/opencv.hpp"

//cv::Mat mTest = cv::imread("/home/austin/Data/idl/images/AFLW/image12783.jpg", 0);
//cv::Mat mTest = cv::imread("/home/austin/Data/Temp/SliceImage/Image/ToBeijing/Second_Send/20170418_video/10AM/2162/1796495694_img/9990.png", 0);

int main(int argc, char **argv)
{
    cv::Mat mTest;

    if (argc != 2){
        std::cout << "Usage: AlphaDet_test image_name" << std::endl;
        return -1;
    }
    else{
        mTest = cv::imread(argv[1], 0);
        //mTest = cv::imread("/home/austin/Data/Temp/SliceImage/Image/ToBeijing/Second_Send/20170418_video/10AM/2162/1796495694_img/9990.png", 0);
    }

    std::vector <std::string> model_names = {
            "./models/face/20160802_40x40_new_smp_tfn0015.lv19.20160801_40x40_release_version_new_dp.fp4.dp10.bin",
            "./models/face/20170426_40x40_height_28_p15_r4_i4_24_0128_R0128_F0050.ln24.fp8_10_4.bin",
            "./models/face/26_0128_R0128_F0050.ln26.fp8_10_4.bin"
    };

    /// Init Detector
    AlphaDet *p_faceAlphaDet = new AlphaDet;
    p_faceAlphaDet->init(model_names);

    ///  Detect the image
    std::vector<std::vector<float>> faces_results;
    p_faceAlphaDet->detect(mTest.cols, mTest.rows, mTest.step, (char *)mTest.data, faces_results);

    std::cout << "faces_results.size(): " << faces_results.size() << std::endl;
    for( int i=0; i<faces_results.size(); i++){
        int face_num = faces_results[i].size() / 5; // 5: left, top, right, bottom, confidence
        std::cout << "face_num:" << face_num << std::endl;
        for( int j=0; j<face_num; j++){
            float left = faces_results[i][0];
            float top = faces_results[i][1];
            float right = faces_results[i][2];
            float bottom = faces_results[i][3];
            float conf = faces_results[i][4];

            std::cout << "left " << left << "top " << top << "right " << right << "bottom " << bottom \
                << "conf " << conf << std::endl;
            cv::rectangle(mTest, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 0, 0));
        }
    }

    cv::imshow("test.jpg", mTest);
    cv::waitKey(0);

    return 0;
}
