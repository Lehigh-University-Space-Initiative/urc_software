/**
 * @file
 * VideoStreamer node: captures the selected rover camera, marks ArUco tags, and publishes the video
 *
 * Runs on: the main computer (10.0.0.10)
 * Started by: main_computer_launch.py (run modes "main_computer" and "hootl")
 *
 * Publishes:
 *   /video_stream (sensor_msgs/Image) - the selected camera, with any ArUco markers drawn on it
 *   /video_stream_3d (sensor_msgs/Image) - a second camera, only while lusi_vision_mode is true
 *
 * Parameters:
 *   stream_cam (int) - which camera to stream (index into cam_map_; the GUI's video panel sets this)
 *   lusi_vision_mode (bool) - also stream the second "3D" camera
 *
 * How it connects to the system:
 *   - image_transport also publishes a compressed copy, /video_stream/compressed, for the radio link
 *   - On the base station, base_station_launch.py decompresses it back into /video_stream/image_raw
 *   - The GUI's Video Stream panel shows that decompressed stream, and LUSIVisionStreamer serves it to LUSI Vision clients
 *   - Off the rover (WSL, hootl) there's no camera, so this node warns every few seconds and publishes nothing
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <chrono>

#include <opencv2/aruco.hpp>

// ArUco markers are printed square barcodes used to mark targets; this uses the 4x4-bit, 50-marker dictionary
// Syntax: cv::Ptr<T> is OpenCV's reference-counted smart pointer (similar to std::shared_ptr)
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

/**
 * Find ArUco markers in a frame and draw their outline, center, and ID onto it
 *
 * Parameters (inputs):
 *   frame - the camera image (modified in place); must not be empty
 *
 * Steps:
 *   1. Detect markers from the 4x4 dictionary
 *   2. Draw OpenCV's standard marker overlay
 *   3. For each marker, draw its edges, a dot at its center, and its ID number
 */
void performArucoDetection(cv::Mat& frame)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

        for (size_t i = 0; i < markerCorners.size(); ++i) {
            const auto& corners = markerCorners[i];
            if (corners.size() == 4) {
                cv::Point topLeft = corners[0];
                cv::Point topRight = corners[1];
                cv::Point bottomRight = corners[2];
                cv::Point bottomLeft = corners[3];

                // Drawing the edges again in green (drawDetectedMarkers already does this; kept for emphasis)
                cv::line(frame, topLeft, topRight, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, topRight, bottomRight, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, bottomRight, bottomLeft, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, bottomLeft, topLeft, cv::Scalar(0, 255, 0), 2);

                // The midpoint of opposite corners is the marker's center (red dot)
                cv::Point center((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
                cv::circle(frame, center, 4, cv::Scalar(0, 0, 255), -1);

                // Writing the marker ID just above its top-left corner
                cv::putText(frame, std::to_string(markerIds[i]), cv::Point(topLeft.x, topLeft.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            }
        }
    }
}

/**
 * Publishes the selected camera's video on a timer
 *
 * Syntax: "class VideoStreamer : public rclcpp::Node" makes this object a ROS node itself
 * That's why it can call create_wall_timer, get_parameter, etc. directly instead of through a node pointer
 */
class VideoStreamer : public rclcpp::Node {
public:
    VideoStreamer() : Node("video_streamer")
    {
        declare_parameter<bool>("lusi_vision_mode", false);
        declare_parameter<int>("stream_cam", 0);

        // Syntax: "this" is a pointer to the current object; image_transport needs it to attach the publisher to this node
        image_pub_ = image_transport::create_publisher(this, "/video_stream");
        image_pub_3d_ = image_transport::create_publisher(this, "/video_stream_3d");

        // Syntax: std::bind(&Class::method, this) makes a callable that runs this->method()
        timer_ = create_wall_timer(std::chrono::milliseconds(60), std::bind(&VideoStreamer::timer_callback, this));  // ~16 Hz

        cam_map_ = {0, 1, 2, 99};  // Linux video device per GUI camera button: Front, Back, Bottom, Arm (99 = no arm camera assigned yet)
        cam_3d_ = 1;  // Device used for the second camera in LUSI Vision mode
        current_streaming_cam_ = -1;  // -1 forces the first timer tick to open a camera
    }

private:
    /**
     * Steps:
     *   1. Read the parameters; if the selected camera changed, open the new one
     *   2. Grab a frame, and if it isn't empty, mark ArUco tags and publish it
     *   3. In LUSI Vision mode, also grab and publish a frame from the second camera
     */
    void timer_callback()
    {
        bool lusi_vision_3d;
        int new_streaming_cam;

        get_parameter("lusi_vision_mode", lusi_vision_3d);
        get_parameter("stream_cam", new_streaming_cam);

        if (new_streaming_cam != current_streaming_cam_) {
            current_streaming_cam_ = new_streaming_cam;

            if ((current_streaming_cam_ < 0) || (current_streaming_cam_ >= cam_map_.size())) {
                RCLCPP_WARN(this->get_logger(), "Invalid camera number selected");
                return;
            }

            int actual_cam = cam_map_[current_streaming_cam_];
            RCLCPP_INFO(this->get_logger(), "Switching to camera %d", actual_cam);
            cap_.open(actual_cam, cv::CAP_V4L2);  // V4L2 is Linux's standard camera interface

            // Not retried automatically; selecting another camera in the GUI and back retries it
            if (!cap_.isOpened()) {
                RCLCPP_WARN(this->get_logger(), "Failed to open camera %d", actual_cam);
                return;
            }
        }

        cv::Mat frame, frame_3d;
        // Syntax: "cap_ >> frame" is OpenCV's operator for reading the next frame (empty if there's no camera)
        cap_ >> frame;

        // Skipping ArUco on an empty frame, since detectMarkers throws on an empty image (no camera attached)
        if (!frame.empty()) {
            performArucoDetection(frame);
        }

        if (lusi_vision_3d) {
            if (!cap_3d_.isOpened()) {
                cap_3d_.open(cam_3d_, cv::CAP_V4L2);
            }
            cap_3d_ >> frame_3d;
        }

        if (!frame.empty()) {
            // Syntax: cv_bridge converts between OpenCV images (cv::Mat) and ROS image messages
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            RCLCPP_DEBUG(this->get_logger(), "Publishing frame");
            image_pub_.publish(*msg);
        }
        else {
            // Throttled to every 5 s so a missing camera doesn't flood the log at the timer rate
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No frame data (no camera connected?)");
        }

        if (!frame_3d.empty()) {
            auto msg_3d = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_3d).toImageMsg();
            image_pub_3d_.publish(*msg_3d);
        }
    }

    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_3d_;

    cv::VideoCapture cap_;  // The selected camera
    cv::VideoCapture cap_3d_;  // The second camera for LUSI Vision mode

    std::vector<int> cam_map_;
    int cam_3d_;
    int current_streaming_cam_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Syntax: spin() runs this node's timer and callbacks until shutdown (Ctrl+C)
    rclcpp::spin(std::make_shared<VideoStreamer>());
    rclcpp::shutdown();

    return 0;
}
