#include <chrono>
#include <memory>
#include <string>
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
        // Initialize OpenCV video writer
        std::string output_file = "output.avi";  // Adjust the file name as needed
        video_writer_ = cv::VideoWriter(output_file, cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(1920, 1080));

        // Initialize RealSense pipeline
        rs2::config cfg;
        cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1920, 1080, rs2_format::RS2_FORMAT_BGR8, 30);
        pipe_.start(cfg);

        // Create a timer to capture frames periodically
        timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() {
            rs2::frameset frames;
            if (pipe_.poll_for_frames(&frames)) {
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

private:
    rs2::pipeline pipe_;
    cv::VideoWriter video_writer_;
    rclcpp::TimerBase::SharedPtr timer_;
    int frame_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseVideoRecorder>());
    rclcpp::shutdown();
    return 0;
}
