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
        this->declare_parameter<bool>("enable_rectification", false);
        this->declare_parameter<std::string>("left_camera_info_url", "");
        this->declare_parameter<std::string>("right_camera_info_url", "");

        video_port_ = this->get_parameter("video_port").as_int();
        fps_ = this->get_parameter("fps").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        vis_cv_image_ = this->get_parameter("vis_cv_image").as_bool();
        enable_rectification_ = this->get_parameter("enable_rectification").as_bool();
        left_camera_info_url_ = this->get_parameter("left_camera_info_url").as_string();
        right_camera_info_url_ = this->get_parameter("right_camera_info_url").as_string();

        left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 10);
        right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 10);
        left_rect_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_rect", 10);
        right_rect_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_rect", 10);
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

        // Set camera params (Static high FPS to minimize motion blur)
        std::string pipeline = "v4l2src device=/dev/video" + std::to_string(video_port_) +
                               " ! image/jpeg, width=" + std::to_string(image_width_) +
                               ", height=" + std::to_string(image_height_) +
                               ", framerate=" + std::to_string(60) + "/1 ! jpegdec ! videoconvert ! appsink"; 

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
        // Start a thread to capture frames at a fixed rate (60 FPS)
        std::thread([this]() {
            rclcpp::Rate capture_rate(60); // Fixed capture rate
        while (rclcpp::ok())
        {
            cv::Mat frame;
            cap_.read(frame);
            if (!frame.empty())
            {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_frame_ = frame.clone(); // Store the latest frame
                }
                capture_rate.sleep();
            }
        }).detach();

        // Start a timer to publish frames at the desired rate (parameter `fps`)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_), // Publish interval based on `fps`
            [this]() {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (!latest_frame_.empty())
                {
                    split_and_publish(latest_frame_); // Publish the latest frame

                if (vis_cv_image_)
                {
                    cv::imshow("Camera", latest_frame_);
                    cv::waitKey(1);
                }
            }
            });
    }

    void split_and_publish(const cv::Mat& frame)
    {
        // Split the image into left and right halves
        cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        cv::Mat right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

        //perform stereo rectification 
        if (enable_rectification_)
        {
            rectify_images(left_image, right_image);
        }
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

        // Synchronize headers with the same timestamp
        auto current_time = this->now();
        left_msg->header.stamp = current_time;
        right_msg->header.stamp = current_time;
        left_camera_info.header.stamp = current_time;
        right_camera_info.header.stamp = current_time;

        // Synchronize frame IDs
        left_camera_info.header.frame_id = left_msg->header.frame_id;
        right_camera_info.header.frame_id = right_msg->header.frame_id;

        // Publish the images and camera info
        left_publisher_->publish(*left_msg);
        right_publisher_->publish(*right_msg);
        left_camera_info_publisher_->publish(left_camera_info);
        right_camera_info_publisher_->publish(right_camera_info);
    }

    void rectify_images(cv::Mat& left_image, cv::Mat& right_image)
    {
    // Get camera info
    auto left_camera_info = left_camera_info_manager_->getCameraInfo();
    auto right_camera_info = right_camera_info_manager_->getCameraInfo();

    // Convert camera info to OpenCV matrices
    cv::Mat left_K = cv::Mat(3, 3, CV_64F, left_camera_info.k.data());
    cv::Mat left_D = cv::Mat(1, left_camera_info.d.size(), CV_64F, left_camera_info.d.data());
    cv::Mat right_K = cv::Mat(3, 3, CV_64F, right_camera_info.k.data());
    cv::Mat right_D = cv::Mat(1, right_camera_info.d.size(), CV_64F, right_camera_info.d.data());

    // Check if calibration data is valid
    if (left_K.empty() || right_K.empty() || left_D.empty() || right_D.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid calibration data. Skipping rectification.");
        return;
    }

    // Extract projection matrices
    cv::Mat left_P = cv::Mat(3, 4, CV_64F, left_camera_info.p.data());
    cv::Mat right_P = cv::Mat(3, 4, CV_64F, right_camera_info.p.data());

    // Extract rotation and translation from projection matrices
    // Extract rotation matrix R from the left projection matrix
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    if (left_P.cols >= 3 && left_P.rows >= 3)
    {
        R = left_P(cv::Rect(0, 0, 3, 3)).clone(); // Directly extract the rotation matrix
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid projection matrix dimensions. Skipping calculation.");
        return;
    }

    // Validate the rotation matrix
    if (cv::norm(R) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Rotation matrix is zero. Check calibration data.");
        return;
    }

    // Extract translation vector T from the right projection matrix
    cv::Mat T = (cv::Mat_<double>(3, 1) << 
                 right_P.at<double>(0, 3) / right_P.at<double>(0, 0), // Tx / fx
                 right_P.at<double>(1, 3) / right_P.at<double>(1, 1), // Ty / fy
                 0); // No translation along z-axis for rectified images

    // Log the rotation and translation
    RCLCPP_INFO(this->get_logger(), "Rotation matrix R: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    RCLCPP_INFO(this->get_logger(), "Translation vector T: [%f, %f, %f]",
                T.at<double>(0), T.at<double>(1), T.at<double>(2));

    if (T.empty() || cv::norm(T) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid translation vector T. Skipping rectification.");
        return;
    }

    // Rectification matrices
    cv::Mat R1, R2, P1, P2, Q;

    // Perform stereo rectification
    cv::stereoRectify(left_K, left_D, right_K, right_D, left_image.size(), R, T, R1, R2, P1, P2, Q);

    // Generate rectification maps
    cv::Mat left_map1, left_map2, right_map1, right_map2;
    cv::initUndistortRectifyMap(left_K, left_D, R1, P1, left_image.size(), CV_32FC1, left_map1, left_map2);
    cv::initUndistortRectifyMap(right_K, right_D, R2, P2, right_image.size(), CV_32FC1, right_map1, right_map2);

    // Check if maps are valid
    if (left_map1.empty() || left_map2.empty() || right_map1.empty() || right_map2.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate rectification maps. Skipping rectification.");
        return;
    }

    // Apply rectification maps
    cv::Mat rectified_left, rectified_right;
    cv::remap(left_image, rectified_left, left_map1, left_map2, cv::INTER_LINEAR);
    cv::remap(right_image, rectified_right, right_map1, right_map2, cv::INTER_LINEAR);

    // Overlay the rectified images
    cv::Mat overlay;
    cv::addWeighted(rectified_left, 0.5, rectified_right, 0.5, 0, overlay);

    // Colorize the overlay for visual distinction
    cv::Mat blue_mask, red_mask;
    cv::cvtColor(rectified_left, blue_mask, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rectified_right, red_mask, cv::COLOR_BGR2GRAY);
    cv::Mat blue_overlay, red_overlay;
    cv::applyColorMap(blue_mask, blue_overlay, cv::COLORMAP_COOL);
    cv::applyColorMap(red_mask, red_overlay, cv::COLORMAP_HOT);

    cv::Mat combined_overlay;
    cv::addWeighted(blue_overlay, 0.5, red_overlay, 0.5, 0, combined_overlay);

    // Display the combined overlay
    cv::imshow("Rectified Overlay", combined_overlay);
    cv::waitKey(1);

    RCLCPP_INFO(this->get_logger(), "Stereo rectification completed successfully.");

    // Publish rectified images
    auto left_rect_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified_left).toImageMsg();
    auto right_rect_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified_right).toImageMsg();

    left_rect_msg->header.frame_id = "left_camera";
    left_rect_msg->header.stamp = this->now();
    right_rect_msg->header.frame_id = "right_camera";
    right_rect_msg->header.stamp = left_rect_msg->header.stamp;

    left_rect_publisher_->publish(*left_rect_msg);
    right_rect_publisher_->publish(*right_rect_msg);

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_publisher_; 
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
    bool enable_rectification_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}