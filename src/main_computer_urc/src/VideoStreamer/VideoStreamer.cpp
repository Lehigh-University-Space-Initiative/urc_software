#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <chrono>

#include <opencv2/aruco.hpp>


// Create a dictionary and parameters for marker detection
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();


void performArucoDetection(cv::Mat& frame) {
    // Perform marker detection
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // Draw detected markers
    if (!markerIds.empty())
    {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

        // Additional processing (like determining the center or drawing more details)
        for (size_t i = 0; i < markerCorners.size(); ++i)
        {
            // Marker corners are automatically provided as a vector of points
            const auto &corners = markerCorners[i];
            if (corners.size() == 4)
            {
                cv::Point topLeft = corners[0];
                cv::Point topRight = corners[1];
                cv::Point bottomRight = corners[2];
                cv::Point bottomLeft = corners[3];

                // Draw lines (optional, since drawDetectedMarkers already does this)
                cv::line(frame, topLeft, topRight, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, topRight, bottomRight, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, bottomRight, bottomLeft, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, bottomLeft, topLeft, cv::Scalar(0, 255, 0), 2);

                // Find and draw the center of the marker
                cv::Point center((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
                cv::circle(frame, center, 4, cv::Scalar(0, 0, 255), -1);

                // Annotate with the marker ID
                cv::putText(frame, std::to_string(markerIds[i]), cv::Point(topLeft.x, topLeft.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            }
        }
    }
}

class VideoStreamer : public rclcpp::Node {
public:
    VideoStreamer() : Node("video_streamer") {
        declare_parameter<bool>("lusi_vision_mode", false);
        declare_parameter<int>("stream_cam", 0);

        image_pub_ = image_transport::create_publisher(this, "/video_stream");
        image_pub_3d_ = image_transport::create_publisher(this, "/video_stream_3d");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(60),  // ~30 Hz
            std::bind(&VideoStreamer::timer_callback, this)
        );

        cam_map_ = {0, 1, 2, 99};
        cam_3d_ = 1;
        current_streaming_cam_ = -1;
    }

private:
    void timer_callback() {
        bool lusi_vision_3d;
        int new_streaming_cam;

        get_parameter("lusi_vision_mode", lusi_vision_3d);
        get_parameter("stream_cam", new_streaming_cam);

        if (new_streaming_cam != current_streaming_cam_) {
            current_streaming_cam_ = new_streaming_cam;

            if (current_streaming_cam_ < 0 || current_streaming_cam_ >= cam_map_.size()) {
                RCLCPP_WARN(this->get_logger(), "Invalid camera number selected");
                return;
            }

            int actual_cam = cam_map_[current_streaming_cam_];
            RCLCPP_INFO(this->get_logger(), "Switching to camera %d", actual_cam);
            cap_.open(actual_cam, cv::CAP_V4L2);

            if (!cap_.isOpened()) {
                RCLCPP_WARN(this->get_logger(), "Failed to open camera %d", actual_cam);
                return;
            }
        }

        cv::Mat frame, frame_3d;
        cap_ >> frame;


        //perform aruco detection
        performArucoDetection(frame);

        if (lusi_vision_3d) {
            if (!cap_3d_.isOpened()) {
                cap_3d_.open(cam_3d_, cv::CAP_V4L2);
            }
            cap_3d_ >> frame_3d;
        }

        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            RCLCPP_WARN(this->get_logger(), "Print testing frame");
            image_pub_.publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "No frame data");
        }

        if (!frame_3d.empty()) {
            auto msg_3d = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_3d).toImageMsg();
            image_pub_3d_.publish(*msg_3d);
        }
    }

    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_3d_;

    cv::VideoCapture cap_;
    cv::VideoCapture cap_3d_;

    std::vector<int> cam_map_;
    int cam_3d_;
    int current_streaming_cam_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamer>());
    rclcpp::shutdown();
    return 0;
}