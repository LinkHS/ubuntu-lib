#include <iostream>
#include "opencv2/opencv.hpp"

int main()
{
    ///  Load a raw image
    int img_w = 512;
    int img_h = 512;
    int img_step = 512;
    std::string image_name = "data/Lenna_512x512x1.raw";
    char *img = NULL;
    std::ifstream ifs(image_name.c_str(), std::ifstream::binary);
    int length;

    if (ifs) {
        std::cout << "Load raw image OK " << image_name << std::endl;
        ifs.seekg(0, ifs.end);
        length = ifs.tellg();
        ifs.seekg(0, ifs.beg);
        if (length != img_w * img_h) {
            printf("incorrect file length!\n");
            return -1;
        }
        img = new char[length];
        ifs.read(img, length);
    }
    else {
        std::cout << "Failed in loading raw image!" << std::endl;
        return -1;
    }

    cv::Mat mRaw(img_h, img_w, CV_8UC1, img);
    cv::imshow("test.jpg", mRaw);
    cv::waitKey(0);
}
