#include "ImageHelper.h"

ImageHelper::ImageHelper(cv::Mat image)
    : image(image)
{
    // if (image.empty()) {
    //     return;
    // }
    // glGenTextures(1, &texture);
    // glBindTexture(GL_TEXTURE_2D, texture);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
    // ROS_INFO("setup image bugg");
    

}

ImageHelper::~ImageHelper()
{
    glDeleteTextures(1, &texture);
}

void ImageHelper::updateImage(cv::Mat image)
{
//     if (texture)
//         glDeleteTextures(1, &texture);
    this->image = image;
}

// ImageHelper::ImageHelper()
//     : image(), texture(0)
// {
// }


void ImageHelper::imguiDrawImage()
{   
    if (texture)
        glDeleteTextures(1, &texture);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

    // glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
    if (!image.empty())
        ImGui::Image( reinterpret_cast<void*>( static_cast<intptr_t>( texture ) ), ImVec2( image.cols, image.rows ) );
}
