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

libguarded::plain_guarded<std::vector<uchar>> lastImage;
libguarded::plain_guarded<std::vector<uchar>> lastImage2;

std::vector<uchar> no_img;

libguarded::plain_guarded<LUSIVIsionGenerator*> gen;

bool stereo = false;

void _proccess_frame(bool right_eye, const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv::Mat img;
    auto im = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::resize(im, img, cv::Size(960, 540));
     RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "print inside process frame");


    std::vector<int> img_opts;
    img_opts.push_back(cv::IMWRITE_JPEG_QUALITY);
    img_opts.push_back(90);
    std::vector<uchar> img_dat;

    cv::imencode(".jpg", img, img_dat, img_opts);
    if (!right_eye) {
    auto last_img_dat = lastImage.lock();
    *last_img_dat = std::move(img_dat);
    } else {
    auto last_img_dat = lastImage2.lock();
    *last_img_dat = std::move(img_dat);
    }
}

void proccess_frame(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  _proccess_frame(false, msg);
}
void proccess_frame2(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"), "temp 2");
  _proccess_frame(true, msg);
}

void run_con(sockpp::tcp_socket sock, bool right_eye);

void run_tcp_server(bool right_eye) {
  std::error_code ec;
  sockpp::tcp_acceptor acc{!right_eye ? 8010 : 8011, 10, ec};

  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("LUSIVisionStreamer"), "Could not create tcp acceptor");
    return;
  }
  while (true) {
    sockpp::inet_address peer;
    if (auto res = acc.accept(&peer); !res) {
      //cerr << "Error accepting incoming connection: " << res.error_message() << endl;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"),
                  "Received a stream connection request from client");
      numStreamClientsConnected++;
      sockpp::tcp_socket sock = res.release();
      std::thread thr(run_con, std::move(sock), right_eye);
      thr.detach();
    }
  }
}

void run_con(sockpp::tcp_socket sock, bool right_eye) {
    rclcpp::Rate loop(24);
    while (true) {
        if (!stereo) {
            auto img = lastImage.lock();
            LUSIStreamHeader head{};
            head.sizeFrame1 = img->size(); //img->rows * img->cols * 1 * 3;
            head.sizeFrame2 = 0;

            sock.write_n(&head, sizeof(head));
            //might be a problem: //count = 0;
            //sock.write_n(&count2,sizeof(count2)); //no second image
            auto res = sock.write_n(img->data(), head.sizeFrame1);

            if (res.value() <= 0) {
                break;
            }
        } else {
        auto img = lastImage.lock();
        auto img2 = lastImage2.lock();

        LUSIStreamHeader head{};
        head.sizeFrame1 = img->size(); //img->rows * img->cols * 1 * 3;
         head.sizeFrame2 = img2->size(); //img->rows * img->cols * 1 * 3;
        RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"),
                    "size1: %d, size2: %d", head.sizeFrame1, head.sizeFrame2);

        sock.write_n(&head, sizeof(head));
        auto res = sock.write_n(img->data(), head.sizeFrame1);
        auto res2 = sock.write_n(img2->data(), head.sizeFrame2);

        if (res.value() <= 0 || res2.value() < 0) {
            break;
        }
        }
        loop.sleep();
  }
  numStreamClientsConnected--;
}

// --------------TELEM----------------

void run_telem_con(sockpp::tcp_socket sock);
void run_telem_tcp_server() {
  std::error_code ec;
  sockpp::tcp_acceptor acc{8009, 10, ec};

  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("LUSIVisionStreamer"), "Could not create tcp acceptor");
    return;
  }
  while (true) {
    sockpp::inet_address peer;
    if (auto res = acc.accept(&peer); !res) {
      // Error accepting connection.
    } else {
      RCLCPP_INFO(rclcpp::get_logger("LUSIVisionStreamer"),
                  "Received a connection request from client");
      numClientsConnected++;
      sockpp::tcp_socket sock = res.release();
      std::thread thr(run_telem_con, std::move(sock));
      thr.detach();
    }
  }
}

void run_telem_con(sockpp::tcp_socket sock) {
    std::vector<char> bytes(10);

    rclcpp::Rate loop(30);
    while (true) {
    LUSIVisionTelem telem;
    {
        auto g = gen.lock();
        telem = (*g)->generate();
    }
    auto res = sock.write_n(&telem, sizeof(telem));
    if (res.value() <= 0) {
        break;
    }
    loop.sleep();
    }
    numClientsConnected--;
}

// --------------------------------------------

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("LUSIVisionStreamer");

    {
        auto g = gen.lock();
        *g = new LUSIVIsionGenerator(node);
    }

    //TOOO: set rate to correct amount
    rclcpp::Rate loop_rate(60);

    RCLCPP_INFO(node->get_logger(), "LUSI Vision Streamer is running");

    RCLCPP_INFO(node->get_logger(), "Setting up video subscribers");
    sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/video_stream/image_raw", rclcpp::QoS(1), proccess_frame);
    sub2 = node->create_subscription<sensor_msgs::msg::Image>(
        "/video_stream2/image_raw", rclcpp::QoS(1), proccess_frame2);

    auto no = cv::imread("/home/urcAssets/LUSNoDownlink.png");
    std::vector<int> img_opts;
    img_opts.push_back(cv::IMWRITE_JPEG_QUALITY);
    img_opts.push_back(10);

    cv::imencode(".jpg", no, no_img, img_opts);
    RCLCPP_INFO(node->get_logger(), "No image size: %d", static_cast<int>(no_img.size()));

    //setup sockets
    sockpp::initialize();

    std::thread acceptor2;
    std::thread acceptor(run_tcp_server, false);
    // if (stereo)
    //    acceptor2 = std::thread(run_tcp_server, true);
    std::thread telem_acceptor(run_telem_tcp_server);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
