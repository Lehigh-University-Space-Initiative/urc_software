#include "VideoViewPanel.h"

void VideoViewPanel::drawBody() {
    for (size_t i = 0; i < cameras.size(); i++) {
        auto cam = cameras[i];
        ImGui::SameLine();

        static float b = 0.8f;
        static float c = 0.5f;
        static int x = 2;

        bool current = i == currentCamIndex;

        if (current) {
            ImGui::PushID(("cambtn: " + std::to_string(i)).c_str());
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(x / 7.0f, b, b));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(x / 7.0f, b, b));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(x / 7.0f, c, c));
        }
        if (ImGui::Button(cam.name.c_str())) {
            newCamIndex = i;
        }

        if (current) {
            ImGui::PopStyleColor(3);
            ImGui::PopID();
        }
    }

    ImGui::Separator();

    if (currentImageHolder) {
        currentImageHolder->imguiDrawImage();
    }
}

void VideoViewPanel::setupSubscriber() {
    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Got frame");
        currentImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
        lastFrameTime = std::chrono::system_clock::now();
        showingLOS = false;
        setNewIMG();
    };

    RCLCPP_INFO(node_->get_logger(), "Setting up video subscriber");
    sub = node_->create_subscription<sensor_msgs::msg::Image>("/videoStream/image_raw", 10, callback);
}

void VideoViewPanel::setNewIMG() {
    cv::cvtColor(currentImage, currentImage, cv::COLOR_BGR2RGBA);
    cv::resize(currentImage, currentImage, cv::Size(960, 540));
    if (currentImageHolder) {
        delete currentImageHolder;
    }
    currentImageHolder = new ImageHelper(currentImage);
}

void VideoViewPanel::setLOSIMG() {
    currentImage = cv::imread(placeholderIMGPath.c_str(), cv::IMREAD_COLOR);
    if (currentImage.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Placeholder image not loaded");
        return;
    }
    setNewIMG();
}

void VideoViewPanel::setup() {
    setLOSIMG();
    setupSubscriber();
}

void VideoViewPanel::update() {
    if (newCamIndex != currentCamIndex) {
        currentCamIndex = newCamIndex;

        // Replace ros::param::set with appropriate ROS 2 parameter setting
        rclcpp::Parameter cam_param("streamCam", static_cast<int>(currentCamIndex));
        node_->set_parameter(cam_param);
    }

    if (!showingLOS) {
        auto now = std::chrono::system_clock::now();
        auto losTime = std::chrono::milliseconds(3000);
        if (now - lastFrameTime >= losTime) {
            showingLOS = true;
            setLOSIMG();
        }
    }
}

VideoViewPanel::~VideoViewPanel() {
    if (currentImageHolder) {
        delete currentImageHolder;
    }
}

VideoViewPanel::Camera::Camera(int id, std::string name)
    : id(id), name(name) {}
