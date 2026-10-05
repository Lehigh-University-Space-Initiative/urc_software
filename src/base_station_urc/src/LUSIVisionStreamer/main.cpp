/**
 * @file
 * LUSIVisionStreamer node: serves rover video and telemetry to LUSI Vision clients over plain TCP sockets
 *
 * Runs on: the base station laptop
 * Started by: base_station_launch.py (run modes "base_station" and "hootl")
 *
 * Subscribes:
 *   /video_stream/image_raw (sensor_msgs/Image) - rover camera (left eye)
 *   /video_stream2/image_raw (sensor_msgs/Image) - second camera for stereo (right eye; nothing publishes it yet)
 *   plus the topics listed in LUSIVisionTelem.cpp for telemetry
 *
 * TCP ports:
 *   8010 - video: repeatedly sends a LUSIStreamHeader followed by the JPEG bytes, ~24 frames per second
 *   8011 - stereo right-eye video (its server is currently disabled)
 *   8009 - telemetry: sends one raw LUSIVisionTelem struct ~30 times per second
 *
 * How it connects to the system:
 *   - LUSI Vision is a separate client application (not in this repo) that connects to these ports
 *   - Each connected client is served by its own detached thread
 */
#include "stdio.h"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

#include <opencv2/opencv.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <cv_bridge/cv_bridge.h>

#include "sockpp/udp_socket.h"
#include "sockpp/tcp_acceptor.h"

#include "cs_plain_guarded.h"
#include <thread>

#include "LUSIVisionTelem.h"

// Latest encoded JPEG for each eye, guarded because the ROS callback writes them while client threads read them
libguarded::plain_guarded<std::vector<uchar>> lastImage;
libguarded::plain_guarded<std::vector<uchar>> lastImage2;

std::vector<uchar> no_img;  // "No downlink" placeholder JPEG (encoded at startup but not currently sent)

libguarded::plain_guarded<LUSIVIsionGenerator*> gen;  // Shared telemetry generator, created in main()

bool stereo = false;  // Send both eyes per frame (the right-eye camera isn't published yet)

/**
 * Shrink a camera frame to 960x540, encode it as JPEG, and store it as the latest image for one eye
 *
 * Parameters (inputs):
 *   right_eye - true to store it as the right eye (stereo), false for the left/only eye
 *   msg - the camera frame
 */
void _proccess_frame(bool right_eye, const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv::Mat img;
    // toCvShare avoids copying the pixels (unlike toCvCopy), which is fine since we only read them here
    auto im = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::resize(im, img, cv::Size(960, 540));
    RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "print inside process frame");

    std::vector<int> img_opts;
    img_opts.push_back(cv::IMWRITE_JPEG_QUALITY);
    img_opts.push_back(90);  // JPEG quality out of 100
    std::vector<uchar> img_dat;

    cv::imencode(".jpg", img, img_dat, img_opts);
    // Syntax: std::move hands the buffer over without copying it (img_dat is left empty)
    if (!right_eye) {
        auto last_img_dat = lastImage.lock();
        *last_img_dat = std::move(img_dat);
    }
    else {
        auto last_img_dat = lastImage2.lock();
        *last_img_dat = std::move(img_dat);
    }
}

void proccess_frame(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    _proccess_frame(false, msg);
}

void proccess_frame2(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "temp 2");
    _proccess_frame(true, msg);
}

// Syntax: a declaration before the definition lets run_tcp_server call run_con, which is defined further down
void run_con(sockpp::tcp_socket sock, bool right_eye);

/**
 * Accept video clients forever, starting one run_con thread per client
 *
 * Parameters (inputs):
 *   right_eye - false serves port 8010 (left/only eye), true serves port 8011 (right eye)
 */
void run_tcp_server(bool right_eye)
{
    std::error_code ec;
    sockpp::tcp_acceptor acc{!right_eye ? 8010 : 8011, 10, ec};  // Listening with a backlog of 10 pending connections

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("LUSIVisionStreamer"), "Could not create tcp acceptor");
        return;
    }
    while (true) {
        sockpp::inet_address peer;
        // Syntax: "if (auto res = ...; !res)" declares res and tests it in one statement (C++17)
        if (auto res = acc.accept(&peer); !res) {
            // Error accepting a connection; keep listening
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "Received a stream connection request from client");
            numStreamClientsConnected++;
            sockpp::tcp_socket sock = res.release();
            std::thread thr(run_con, std::move(sock), right_eye);
            thr.detach();
        }
    }
}

/**
 * Send video frames to one client until it disconnects
 *
 * Steps:
 *   1. Each pass, send a header with the JPEG size(s), then the JPEG bytes (one eye, or both in stereo mode)
 *   2. Stop when a write fails (the client went away), and decrement the client count
 */
void run_con(sockpp::tcp_socket sock, bool right_eye)
{
    rclcpp::Rate loop(24);  // ~24 frames per second
    while (true) {
        if (!stereo) {
            auto img = lastImage.lock();
            LUSIStreamHeader head{};
            head.sizeFrame1 = img->size();
            head.sizeFrame2 = 0;  // No second image

            sock.write_n(&head, sizeof(head));
            auto res = sock.write_n(img->data(), head.sizeFrame1);

            if (res.value() <= 0) {
                break;
            }
        }
        else {
            auto img = lastImage.lock();
            auto img2 = lastImage2.lock();

            LUSIStreamHeader head{};
            head.sizeFrame1 = img->size();
            head.sizeFrame2 = img2->size();
            RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "size1: %lu, size2: %lu", (unsigned long)head.sizeFrame1, (unsigned long)head.sizeFrame2);

            sock.write_n(&head, sizeof(head));
            auto res = sock.write_n(img->data(), head.sizeFrame1);
            auto res2 = sock.write_n(img2->data(), head.sizeFrame2);

            if ((res.value() <= 0) || (res2.value() < 0)) {
                break;
            }
        }
        loop.sleep();
    }
    numStreamClientsConnected--;
}

void run_telem_con(sockpp::tcp_socket sock);

/// Accept telemetry clients on port 8009 forever, starting one run_telem_con thread per client
void run_telem_tcp_server()
{
    std::error_code ec;
    sockpp::tcp_acceptor acc{8009, 10, ec};

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("LUSIVisionStreamer"), "Could not create tcp acceptor");
        return;
    }
    while (true) {
        sockpp::inet_address peer;
        if (auto res = acc.accept(&peer); !res) {
            // Error accepting a connection; keep listening
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "Received a connection request from client");
            numClientsConnected++;
            sockpp::tcp_socket sock = res.release();
            std::thread thr(run_telem_con, std::move(sock));
            thr.detach();
        }
    }
}

/// Send a telemetry packet to one client ~30 times a second until it disconnects
void run_telem_con(sockpp::tcp_socket sock)
{
    std::vector<char> bytes(10);  // Unused

    rclcpp::Rate loop(30);
    while (true) {
        LUSIVisionTelem telem;
        {
            auto g = gen.lock();
            telem = (*g)->generate();
        }
        // Sending the struct's raw bytes (the client must use the same struct layout)
        auto res = sock.write_n(&telem, sizeof(telem));
        if (res.value() <= 0) {
            break;
        }
        loop.sleep();
    }
    numClientsConnected--;
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2;

/**
 * Steps:
 *   1. Start ROS, create the node and the telemetry generator, and subscribe to both video topics
 *   2. Pre-encode the "no downlink" placeholder
 *   3. Start the video and telemetry socket servers on background threads
 *   4. Process ROS callbacks at 60 Hz until shutdown
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("LUSIVisionStreamer");

    {
        auto g = gen.lock();
        *g = new LUSIVIsionGenerator(node);
    }

    // TODO: set rate to correct amount
    rclcpp::Rate loop_rate(60);

    RCLCPP_INFO(node->get_logger(), "LUSI Vision Streamer is running");

    RCLCPP_INFO(node->get_logger(), "Setting up video subscribers");
    // QoS(1) keeps only the newest frame, since older frames are useless for live video
    sub = node->create_subscription<sensor_msgs::msg::Image>("/video_stream/image_raw", rclcpp::QoS(1), proccess_frame);
    sub2 = node->create_subscription<sensor_msgs::msg::Image>("/video_stream2/image_raw", rclcpp::QoS(1), proccess_frame2);

    auto no = cv::imread("/home/urcAssets/LUSNoDownlink.png");
    std::vector<int> img_opts;
    img_opts.push_back(cv::IMWRITE_JPEG_QUALITY);
    img_opts.push_back(10);  // JPEG quality out of 100

    cv::imencode(".jpg", no, no_img, img_opts);
    RCLCPP_INFO(node->get_logger(), "No image size: %d", static_cast<int>(no_img.size()));

    sockpp::initialize();

    // The servers loop forever, so the threads are detached rather than joined
    // Destroying a joinable std::thread at exit calls std::terminate, which used to print an error on every Ctrl+C
    std::thread acceptor(run_tcp_server, false);
    acceptor.detach();
    std::thread telem_acceptor(run_telem_tcp_server);
    telem_acceptor.detach();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
