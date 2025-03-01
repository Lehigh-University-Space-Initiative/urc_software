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
        //RCLCPP_INFO(node_->get_logger(), "Got frame");
        currentImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
        lastFrameTime = std::chrono::system_clock::now();
        showingLOS = false;
        setNewIMG();
    };

    RCLCPP_INFO(node_->get_logger(), "Setting up video subscriber");
    sub = node_->create_subscription<sensor_msgs::msg::Image>("/video_stream/image_raw", 10, callback);
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
    //setup parameter setting
    camParamClient_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,"VideoStreamer");

    // Wait for the service to become available
    if (!camParamClient_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "SetParameters service not available");
      return;
    }
}

void VideoViewPanel::update() {
    if (newCamIndex != currentCamIndex) {
        currentCamIndex = newCamIndex;

        // Replace ros::param::set with appropriate ROS 2 parameter setting
        rclcpp::Parameter cam_param("stream_cam", static_cast<int>(currentCamIndex));
        camParamClient_->set_parameters({cam_param},
                                [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
                                {
                                    future.wait();
                                    auto results = future.get();
                                    if (results.size() != 1)
                                    {
                                        RCLCPP_ERROR_STREAM(node_->get_logger(), "expected 1 result, got " << results.size());
                                    }
                                    else
                                    {
                                        if (results[0].successful)
                                        {
                                            RCLCPP_INFO(node_->get_logger(), "success");
                                        }
                                        else
                                        {
                                            RCLCPP_ERROR(node_->get_logger(), "failure");
                                        }
                                    }
                                }
        );
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
