/**
 * @file
 * Shows an OpenCV image inside an ImGui window by uploading it to an OpenGL texture
 */
#pragma once

#include "GUI.h"

#include <opencv2/opencv.hpp>

class ImageHelper {
    // Syntax: members listed before "public:" in a class are private
    cv::Mat image;  // The image to show (must be RGBA, 8 bits per channel)
    GLuint texture = 0;  // OpenGL texture handle (0 means none created yet)

public:
    /**
     * Parameters (inputs):
     *   image - the image to show; must be in RGBA format
     */
    ImageHelper(cv::Mat image);
    ~ImageHelper();

    /// Replace the image shown on the next draw
    void updateImage(cv::Mat image);

    /// Upload the image to the GPU and draw it at its full size in the current ImGui window
    void imguiDrawImage();
};
