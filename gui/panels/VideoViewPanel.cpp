#include "VideoViewPanel.h"


void VideoViewPanel::drawBody()
{
    for (size_t i = 0; i < cameras.size(); i++)
    {
        auto cam = cameras[i];
        ImGui::SameLine();

        static float b = 0.8f;//1.0f; //  test whatever color you need from imgui_demo.cpp e.g.
        static float c = 0.5f; // 
        static int x = 2;

        bool current = i == currentCamIndex;

        if (current) {
            ImGui::PushID(("cambtn: " + std::to_string(i)).c_str());
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(x/7.0f, b, b));
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

    currentImageHolder->imguiDrawImage(); 


}

void VideoViewPanel::setupSubscriber()
{
    auto f = boost::function<void(const sensor_msgs::ImageConstPtr& msg)>([this](auto msg){
    //    ROS_INFO("got frame");
       currentImage = cv_bridge::toCvShare(msg, "bgr8")->image;
       auto now = std::chrono::system_clock::now();
       //ROS_INFO(("Frame Delta: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrameTime).count())).c_str());
       lastFrameTime = now;
        showingLOS = false;
        setNewIMG(); 
    });
    ROS_INFO("setup vid sub");
    sub = n.subscribe("/videoStream/image_raw", 1, f);
    //sub = n.subscribe("/videoStreamAR", 1, f);
}

void VideoViewPanel::setNewIMG()
{
    cv::cvtColor(currentImage, currentImage, cv::COLOR_BGR2RGBA);
    cv::resize(currentImage, currentImage, cv::Size(960, 540));
    if (currentImageHolder)
        // currentImageHolder->updateImage(currentImage);
        delete currentImageHolder;
    // else {
        currentImageHolder = new ImageHelper(currentImage);
    // }
}

void VideoViewPanel::setLOSIMG()
{
    currentImage = cv::imread(placeholderIMGPath.c_str(), cv::IMREAD_COLOR);
    if (currentImage.empty())
    {
        ROS_ERROR("Placeholder image not loaded");
        return;
    }
    setNewIMG();
}

void VideoViewPanel::setup()
{
    setLOSIMG();
    setupSubscriber();
}

void VideoViewPanel::update()
{
    if (newCamIndex != currentCamIndex) {
        currentCamIndex = newCamIndex;

        ros::param::set("/streamCam",static_cast<int>(currentCamIndex));
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

VideoViewPanel::~VideoViewPanel()
{
    if (currentImageHolder)
        delete currentImageHolder;
}

VideoViewPanel::Camera::Camera(int id, std::string name)
    : id(id), name(name)
{
}
