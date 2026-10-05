/**
 * @file
 * Video Stream panel: shows the rover camera feed and lets the operator pick which camera streams
 */
#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "../ImageHelper.h"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoViewPanel : public Panel {
public:
    VideoViewPanel(const std::string& name, const rclcpp::Node::SharedPtr& node)
        : Panel(name, node)
    {
    }

    // Client for setting VideoStreamer's stream_cam parameter on the main computer
    std::shared_ptr<rclcpp::AsyncParametersClient> camParamClient_;
    /// Load the "no downlink" placeholder, subscribe to the video, and connect to VideoStreamer
    virtual void setup() override;
    /// Send camera changes to VideoStreamer, and fall back to the placeholder after 3 s without frames
    virtual void update() override;
    virtual ~VideoViewPanel();

protected:
    struct Camera {
        int id;
        std::string name;

        Camera(int id, std::string name);
    };

    // The order here must match VideoStreamer's cam_map_ (button index = stream_cam value)
    const std::array<Camera, 4> cameras = {Camera(0, "Front"), Camera(1, "Back"), Camera(2, "Bottom"), Camera(3, "Arm")};

    // Installed into the image by the Dockerfile (COPY urcAssets /home/urcAssets)
    const std::string placeholderIMGPath = "/home/urcAssets/LUSNoDownlink.png";

    // Syntax: size_t is unsigned, so -1 wraps to the largest value; it just means "no camera selected yet"
    size_t currentCamIndex = -1;
    size_t newCamIndex = 0;  // Camera the operator picked (sent to VideoStreamer in update())
    cv::Mat currentImage;
    ImageHelper* currentImageHolder = nullptr;  // Owned; replaced for every new frame

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

    std::chrono::system_clock::time_point lastFrameTime;
    bool showingLOS = true;  // True while the placeholder is shown instead of video

    virtual void drawBody() override;

    void setupSubscriber();
    /// Convert currentImage to RGBA at 960x540 and hand it to a new ImageHelper
    void setNewIMG();
    /// Show the "no downlink" placeholder image
    void setLOSIMG();
};
