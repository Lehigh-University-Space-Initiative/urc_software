#pragma once
#include "../Panel.h"
#include "../GUI.h"
#include <chrono>
#include <thread>
#include <mutex>
#include "../ImageHelper.h"



class VideoViewPanel: public Panel, public rclcpp::Node{
public:
    using Panel::Panel;

    virtual void setup() override;
    virtual void update() override;
    virtual ~VideoViewPanel();

protected:
    struct Camera {
        int id;
        std::string name;

        Camera(int id, std::string name);
    };

    const std::array<Camera,4> cameras = {Camera(0,"Front"),Camera(1,"Back"),Camera(2,"Bottom"),Camera(3,"Arm")};

    const std::string placeholderIMGPath = "/home/urcAssets/LUSNoDownlink.png";
    // const std::string placeholderIMGPath = "/home/urcAssets/images.jpeg";

    size_t currentCamIndex = -1;
    size_t newCamIndex = 0;
    cv::Mat currentImage;
    ImageHelper* currentImageHolder = nullptr;

    //models:

    // ros::Subscriber sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    // ros::NodeHandle n;

    std::chrono::system_clock::time_point lastFrameTime;
    bool showingLOS = true;

    virtual void drawBody();

    void setupSubscriber();

    void setNewIMG();
    void setLOSIMG();

};