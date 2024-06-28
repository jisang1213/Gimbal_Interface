#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>

class RealSenseVideoRecorder : public rclcpp::Node
{
public:
    RealSenseVideoRecorder()
        : Node("realsense_video_recorder"), frame_count_(0)
    {
        // Generate timestamp for filename
        std::time_t now = std::time(nullptr);
        std::tm tm = *std::localtime(&now);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        std::string timestamp = ss.str();

        // Construct filename with timestamp
        std::string output_file = timestamp + ".mp4";  // Example filename format: output_2024-06-28_15-30-00.mp4

        // Initialize OpenCV video writer with MP4 codec
        video_writer_ = cv::VideoWriter(output_file, cv::VideoWriter::fourcc('H','2','6','4'), 30, cv::Size(1920, 1080));

        // Initialize RealSense pipeline
        rs2::config cfg;
        cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1920, 1080, rs2_format::RS2_FORMAT_BGR8, 30);
        pipe_.start(cfg);

        // Start a separate thread for frame capturing
        capture_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                rs2::frameset frames = pipe_.wait_for_frames();
                auto frame = frames.get_color_frame();
                const int w = frame.as<rs2::video_frame>().get_width();
                const int h = frame.as<rs2::video_frame>().get_height();
                cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

                // Resize the display window to a smaller size
                cv::Mat smaller_image;
                cv::resize(image, smaller_image, cv::Size(), 0.5, 0.5);  // Adjust scaling factor as needed
                cv::imshow("RealSense Video", smaller_image);

                cv::waitKey(1);  // Needed to update display

                video_writer_.write(image);
                frame_count_++;
                RCLCPP_INFO(this->get_logger(), "Saved frame %d", frame_count_);
            }
        });
    }

    ~RealSenseVideoRecorder()
    {
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        video_writer_.release();
    }

private:
    rs2::pipeline pipe_;
    cv::VideoWriter video_writer_;
    std::thread capture_thread_;
    int frame_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseVideoRecorder>());
    rclcpp::shutdown();
    return 0;
}
