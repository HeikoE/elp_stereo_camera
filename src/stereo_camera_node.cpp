#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode() : Node("stereo_camera_node")
    {
        this->declare_parameter<int>("video_port", 6);
        this->declare_parameter<int>("fps", 60);
        this->declare_parameter<int>("image_width", 1600);
        this->declare_parameter<int>("image_height", 600);
        this->declare_parameter<bool>("vis_cv_image", true);
        this->declare_parameter<std::string>("left_camera_info_url", "");
        this->declare_parameter<std::string>("right_camera_info_url", "");

        video_port_ = this->get_parameter("video_port").as_int();
        fps_ = this->get_parameter("fps").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        vis_cv_image_ = this->get_parameter("vis_cv_image").as_bool();
        left_camera_info_url_ = this->get_parameter("left_camera_info_url").as_string();
        right_camera_info_url_ = this->get_parameter("right_camera_info_url").as_string();

        left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 10);
        right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 10);
        left_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 10);
        right_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 10);

        left_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "left_camera", left_camera_info_url_);
        right_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "right_camera", right_camera_info_url_);

        left_set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "left_camera/set_camera_info",
            std::bind(&StereoCameraNode::set_left_camera_info, this, std::placeholders::_1, std::placeholders::_2)
        );

        right_set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "right_camera/set_camera_info",
            std::bind(&StereoCameraNode::set_right_camera_info, this, std::placeholders::_1, std::placeholders::_2)
        );

        std::string pipeline = "v4l2src device=/dev/video" + std::to_string(video_port_) +
                               " ! image/jpeg, width=" + std::to_string(image_width_) +
                               ", height=" + std::to_string(image_height_) +
                               ", framerate=" + std::to_string(fps_) + "/1 ! jpegdec ! videoconvert ! appsink";

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (cap_.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Opened video capture on port %d with GStreamer pipeline", video_port_);

            check_camera_settings();

            if (vis_cv_image_)
            {
                cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
            }

            RCLCPP_INFO(this->get_logger(), "START STREAMING");

            capture_loop();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video capture on port %d with GStreamer pipeline", video_port_);
        }
    }

    ~StereoCameraNode()
    {
        if (vis_cv_image_)
        {
            cv::destroyWindow("Camera");
        }
    }

private:
    void check_camera_settings()
    {
        // Print out the current camera settings
        double actual_fps = cap_.get(cv::CAP_PROP_FPS);
        double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

        RCLCPP_INFO(this->get_logger(), "Camera settings:");
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f", actual_fps);
        RCLCPP_INFO(this->get_logger(), "Resolution: %.0f x %.0f", actual_width, actual_height);
    }

    void capture_loop()
    {
        rclcpp::Rate rate(fps_);
        while (rclcpp::ok())
        {
            cv::Mat frame;
            cap_.read(frame);
            if (!frame.empty())
            {
                split_and_publish(frame);

                if (vis_cv_image_)
                {
                    cv::imshow("Camera", frame);
                    cv::waitKey(1);
                }
            }
            rate.sleep();
        }
    }

    void split_and_publish(const cv::Mat& frame)
    {
        // Split the image into left and right halves
        cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        cv::Mat right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

        // Convert the images to ROS messages
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_image).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_image).toImageMsg();

        // Add information to the headers
        left_msg->header.frame_id = "left_camera";
        left_msg->header.stamp = this->now();
        right_msg->header.frame_id = "right_camera";
        right_msg->header.stamp = this->now();

        // Get the camera info
        sensor_msgs::msg::CameraInfo left_camera_info = left_camera_info_manager_->getCameraInfo();
        sensor_msgs::msg::CameraInfo right_camera_info = right_camera_info_manager_->getCameraInfo();

        // Synchronize headers
        left_camera_info.header = left_msg->header;
        right_camera_info.header = right_msg->header;

        // Publish the images and camera info
        left_publisher_->publish(*left_msg);
        right_publisher_->publish(*right_msg);
        left_camera_info_publisher_->publish(left_camera_info);
        right_camera_info_publisher_->publish(right_camera_info);
    }

    void set_left_camera_info(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
                              std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        if (left_camera_info_manager_->setCameraInfo(request->camera_info))
        {
            response->success = true;
            response->status_message = "Left camera info set successfully.";
        }
        else
        {
            response->success = false;
            response->status_message = "Failed to set left camera info.";
        }
    }

    void set_right_camera_info(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
                               std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        if (right_camera_info_manager_->setCameraInfo(request->camera_info))
        {
            response->success = true;
            response->status_message = "Right camera info set successfully.";
        }
        else
        {
            response->success = false;
            response->status_message = "Failed to set right camera info.";
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_publisher_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> left_camera_info_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> right_camera_info_manager_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr left_set_camera_info_service_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr right_set_camera_info_service_;
    cv::VideoCapture cap_;
    int video_port_;
    int fps_;
    int image_width_;
    int image_height_;
    bool vis_cv_image_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}