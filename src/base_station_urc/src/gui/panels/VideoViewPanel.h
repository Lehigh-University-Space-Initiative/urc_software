#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "../ImageHelper.h"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoViewPanel: public Panel {
public:
    VideoViewPanel(const std::string &name, const rclcpp::Node::SharedPtr &node)
        : Panel(name, node) {}

    std::shared_ptr<rclcpp::AsyncParametersClient> camParamClient_;
    virtual void setup() override;
    virtual void update() override;
    virtual ~VideoViewPanel();

protected:
    struct Camera {
        int id;
        std::string name;

        Camera(int id, std::string name);
    };

    const std::array<Camera, 4> cameras = {Camera(0, "Front"), Camera(1, "Back"), Camera(2, "Bottom"), Camera(3, "Arm")};

    const std::string placeholderIMGPath = "/home/urcAssets/LUSNoDownlink.png";

    size_t currentCamIndex = -1;
    size_t newCamIndex = 0;
    cv::Mat currentImage;
    ImageHelper* currentImageHolder = nullptr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

    std::chrono::system_clock::time_point lastFrameTime;
    bool showingLOS = true;

    virtual void drawBody() override;

    void setupSubscriber();
    void setNewIMG();
    void setLOSIMG();
};
