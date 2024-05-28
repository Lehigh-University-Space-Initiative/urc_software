#pragma once 

#include "GUI.h"

#include <opencv2/opencv.hpp>

class ImageHelper {
    cv::Mat image;
    GLuint texture = 0;
public:
    //image must be in RGBA
    ImageHelper(cv::Mat image);
    // ImageHelper();
    ~ImageHelper();

    void updateImage(cv::Mat image);
    
    void imguiDrawImage();
};