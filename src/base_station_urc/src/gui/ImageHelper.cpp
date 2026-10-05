/**
 * @file
 * Implementation of ImageHelper
 */
#include "ImageHelper.h"

ImageHelper::ImageHelper(cv::Mat image)
    : image(image)
{
}

ImageHelper::~ImageHelper()
{
    glDeleteTextures(1, &texture);  // Freeing the texture's GPU memory
}

void ImageHelper::updateImage(cv::Mat image)
{
    this->image = image;
}

/**
 * Steps:
 *   1. Delete last frame's texture and create a fresh one (the texture is re-uploaded every frame)
 *   2. Use linear filtering so the image looks smooth when scaled
 *   3. Copy the image's pixels to the GPU, then tell ImGui to draw that texture
 */
void ImageHelper::imguiDrawImage()
{
    if (texture) {
        glDeleteTextures(1, &texture);
    }
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Rows are tightly packed (no padding between them)

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
    if (!image.empty()) {
        // Syntax: ImGui identifies textures by an opaque pointer, so the integer handle is cast to one
        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(texture)), ImVec2(image.cols, image.rows));
    }
}
