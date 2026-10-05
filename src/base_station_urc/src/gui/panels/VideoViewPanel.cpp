/**
 * @file
 * Implementation of the Video Stream panel
 *
 * Subscribes:
 *   /video_stream/image_raw (sensor_msgs/Image) - the rover camera, decompressed by base_station_launch.py's video_decompress node
 *
 * Sets remotely:
 *   VideoStreamer's stream_cam parameter (on the main computer) when a camera button is clicked
 */
#include "VideoViewPanel.h"

void VideoViewPanel::drawBody()
{
    for (size_t i = 0; i < cameras.size(); i++) {
        auto cam = cameras[i];
        ImGui::SameLine();

        // Highlight colors for the selected camera's button (HSV: hue x/7, saturation and value b or c)
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

void VideoViewPanel::setupSubscriber()
{
    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        // Converting the ROS image to an OpenCV image (copied, since the message is freed after the callback)
        currentImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
        lastFrameTime = std::chrono::system_clock::now();
        showingLOS = false;
        setNewIMG();
    };

    RCLCPP_INFO(node_->get_logger(), "Setting up video subscriber");
    sub = node_->create_subscription<sensor_msgs::msg::Image>("/video_stream/image_raw", 10, callback);
}

void VideoViewPanel::setNewIMG()
{
    // OpenCV images are BGR (blue first); OpenGL textures here expect RGBA
    cv::cvtColor(currentImage, currentImage, cv::COLOR_BGR2RGBA);
    cv::resize(currentImage, currentImage, cv::Size(960, 540));
    if (currentImageHolder) {
        delete currentImageHolder;
    }
    currentImageHolder = new ImageHelper(currentImage);
}

void VideoViewPanel::setLOSIMG()
{
    currentImage = cv::imread(placeholderIMGPath.c_str(), cv::IMREAD_COLOR);
    if (currentImage.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Placeholder image not loaded");
        return;
    }
    setNewIMG();
}

/**
 * Steps:
 *   1. Show the placeholder until video arrives, and subscribe to the video topic
 *   2. Create the parameter client for VideoStreamer and wait briefly for it
 */
void VideoViewPanel::setup()
{
    setLOSIMG();
    setupSubscriber();

    // Targets the node named "VideoStreamer" (the name main_computer_launch.py gives VideoStreamer_node)
    camParamClient_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "VideoStreamer");

    // This runs before the window opens, so a long wait delays the whole GUI when the rover is off (it used to be 10 s)
    if (!camParamClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "VideoStreamer not reachable yet; camera buttons work once the main computer is up");
    }
}

/**
 * Steps:
 *   1. If the operator picked a different camera, set VideoStreamer's stream_cam to it
 *   2. If video was showing but no frame has arrived for 3 s, switch to the placeholder (loss of signal)
 */
void VideoViewPanel::update()
{
    if (newCamIndex != currentCamIndex) {
        currentCamIndex = newCamIndex;

        rclcpp::Parameter cam_param("stream_cam", static_cast<int>(currentCamIndex));
        camParamClient_->set_parameters({cam_param},
                                        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                                            future.wait();
                                            auto results = future.get();
                                            if (results.size() != 1) {
                                                RCLCPP_ERROR_STREAM(node_->get_logger(), "expected 1 result, got " << results.size());
                                            }
                                            else {
                                                if (results[0].successful) {
                                                    RCLCPP_INFO(node_->get_logger(), "success");
                                                }
                                                else {
                                                    RCLCPP_ERROR(node_->get_logger(), "failure");
                                                }
                                            }
                                        });
    }

    if (!showingLOS) {
        auto now = std::chrono::system_clock::now();
        auto losTime = std::chrono::milliseconds(3000);
        if ((now - lastFrameTime) >= losTime) {
            showingLOS = true;
            setLOSIMG();
        }
    }
}

VideoViewPanel::~VideoViewPanel()
{
    if (currentImageHolder) {
        delete currentImageHolder;
    }
}

VideoViewPanel::Camera::Camera(int id, std::string name)
    : id(id), name(name)
{
}
