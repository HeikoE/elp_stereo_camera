#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <mutex>
#include <thread>

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode() : Node("stereo_camera_node")
    {
        loadParameters();
        initializePublishers();
        initializeServices();
        initiliazeRectificationMaps();
        initializeCamera();

        if (cap_.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Camera initialized successfully.");
            logCameraSettings();

            if (vis_raw_cam_img_)
            {
                cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
            }

            startCaptureLoop();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video capture on port %d.", video_port_);
        }
    }

    ~StereoCameraNode()
    {
        if (vis_raw_cam_img_)
        {
            cv::destroyWindow("Camera");
        }
    }

private:
    void loadParameters()
    {
        this->declare_parameter<int>("video_port", 6);
        this->declare_parameter<int>("fps", 60);
        this->declare_parameter<int>("image_width", 1600);
        this->declare_parameter<int>("image_height", 600);
        this->declare_parameter<bool>("vis_raw_cam_img", true);
        this->declare_parameter<bool>("vis_rect_img_overlay", true);
        this->declare_parameter<bool>("publish_raw_images", true);
        this->declare_parameter<bool>("publish_rectified_images", true);
        this->declare_parameter<std::string>("left_camera_info_url", "");
        this->declare_parameter<std::string>("right_camera_info_url", "");
        this->declare_parameter<bool>("minimize_motion_blur", false);

        video_port_ = this->get_parameter("video_port").as_int();
        fps_ = this->get_parameter("fps").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        vis_raw_cam_img_ = this->get_parameter("vis_raw_cam_img").as_bool();
        vis_rect_img_overlay_ = this->get_parameter("vis_rect_img_overlay").as_bool();
        publish_raw_images_ = this->get_parameter("publish_raw_images").as_bool();
        publish_rectified_images_ = this->get_parameter("publish_rectified_images").as_bool();
        left_camera_info_url_ = this->get_parameter("left_camera_info_url").as_string();
        right_camera_info_url_ = this->get_parameter("right_camera_info_url").as_string();
        minimize_motion_blur_ = this->get_parameter("minimize_motion_blur").as_bool();
    }

    void initializePublishers()
    {
        left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 10);
        right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 10);
        left_rect_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_rect", 10);
        right_rect_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_rect", 10);
        left_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 10);
        right_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 10);
    }

    void initializeServices()
    {
        left_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "left_camera", left_camera_info_url_);
        right_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "right_camera", right_camera_info_url_);

        left_set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "left_camera/set_camera_info",
            std::bind(&StereoCameraNode::setCameraInfo, this, std::placeholders::_1, std::placeholders::_2, left_camera_info_manager_)
        );

        right_set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "right_camera/set_camera_info",
            std::bind(&StereoCameraNode::setCameraInfo, this, std::placeholders::_1, std::placeholders::_2, right_camera_info_manager_)
        );
    }

    void initializeCamera()
    {
        int fps = minimize_motion_blur_ ? 60 : fps_;

        std::string pipeline = "v4l2src device=/dev/video" + std::to_string(video_port_) +
                               " ! image/jpeg, width=" + std::to_string(image_width_) +
                               ", height=" + std::to_string(image_height_) +
                               ", framerate=" + std::to_string(fps) + "/1 ! jpegdec ! videoconvert ! appsink";

        cap_.open(pipeline, cv::CAP_GSTREAMER);
    }

    void logCameraSettings()
    {
        double actual_fps = cap_.get(cv::CAP_PROP_FPS);
        double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

        RCLCPP_INFO(this->get_logger(), "Camera settings:");
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f", actual_fps);
        RCLCPP_INFO(this->get_logger(), "Resolution: %.0f x %.0f", actual_width, actual_height);
    }

    void initiliazeRectificationMaps()
    {
        // Get camera info
        auto left_camera_info = left_camera_info_manager_->getCameraInfo();
        auto right_camera_info = right_camera_info_manager_->getCameraInfo();

        // Get image size
        int image_width = left_camera_info.width;
        int image_height = left_camera_info.height;
        cv::Size image_size(image_width, image_height);

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

        // Extract rotation matrix R from the left projection matrix
        cv::Mat R = cv::Mat(3, 3, CV_64F);
        if (left_P.cols >= 3 && left_P.rows >= 3)
        {
            R = left_P(cv::Rect(0, 0, 3, 3)).clone();
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

        // Perform stereo rectification
        cv::Mat R1, R2, P1, P2, Q;
        cv::stereoRectify(left_K, left_D, right_K, right_D, image_size, R, T, R1, R2, P1, P2, Q);

        // Generate rectification maps
        cv::initUndistortRectifyMap(left_K, left_D, R1, P1, image_size, CV_32FC1, left_map1_, left_map2_);
        cv::initUndistortRectifyMap(right_K, right_D, R2, P2, image_size, CV_32FC1, right_map1_, right_map2_);

        // Check if maps are valid
        if (left_map1_.empty() || left_map2_.empty() || right_map1_.empty() || right_map2_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate rectification maps. Skipping rectification.");
            return;
        }
    }

    void startCaptureLoop()
    {
        std::thread([this]() {
            rclcpp::Rate capture_rate(fps_);
            while (rclcpp::ok())
            {
                cv::Mat frame;
                cap_.read(frame);
                if (!frame.empty())
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_frame_ = frame.clone();
                }
                capture_rate.sleep();
            }
        }).detach();

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_),
            [this]() {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (!latest_frame_.empty())
                {
                    processFrame(latest_frame_);
                }
            });
    }

    void processFrame(const cv::Mat& frame)
    {
        cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        cv::Mat right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

        auto current_time = this->now();

        if (publish_raw_images_)
        {
            publishImage(left_image, "left_camera", left_publisher_, current_time);
            publishImage(right_image, "right_camera", right_publisher_, current_time);
        }

        if(publish_rectified_images_ || vis_rect_img_overlay_)
        {
            rectifyImages(left_image, right_image);
        }
        
        if (publish_rectified_images_)
        {
            publishImage(rectified_left_, "left_camera", left_rect_publisher_, current_time);
            publishImage(rectified_right_, "right_camera", right_rect_publisher_, current_time);
        }

        if(publish_rectified_images_ || publish_raw_images_)
        {
            publishCameraInfo(left_camera_info_manager_->getCameraInfo(), "left_camera", left_camera_info_publisher_, current_time);
            publishCameraInfo(right_camera_info_manager_->getCameraInfo(), "right_camera", right_camera_info_publisher_, current_time);
        }

        if(vis_rect_img_overlay_)
        {
            cv::Mat overlay = overlayStereoImagePair(rectified_left_, rectified_right_);
            cv::imshow("Rectified Overlay", overlay);
            cv::waitKey(1);
        }
        
        if (vis_raw_cam_img_)
        {
            cv::imshow("Camera", frame);
            cv::waitKey(1);
        }
    }

    void rectifyImages(cv::Mat& left_image, cv::Mat& right_image)
    {
        cv::remap(left_image, rectified_left_, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(right_image, rectified_right_, right_map1_, right_map2_, cv::INTER_LINEAR);
    }

    void publishImage(const cv::Mat& image, const std::string& frame_id,
                      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
                      const rclcpp::Time& timestamp)
    {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.frame_id = frame_id;
        msg->header.stamp = timestamp;
        publisher->publish(*msg);
    }

    void publishCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info, const std::string& frame_id,
                           rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher,
                           const rclcpp::Time& timestamp)
    {
        auto info = camera_info;
        info.header.frame_id = frame_id;
        info.header.stamp = timestamp;
        publisher->publish(info);
    }

    void setCameraInfo(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
                       std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response,
                       std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager)
    {
        if (camera_info_manager->setCameraInfo(request->camera_info))
        {
            response->success = true;
            response->status_message = "Camera info set successfully.";
        }
        else
        {
            response->success = false;
            response->status_message = "Failed to set camera info.";
        }
    }

    cv::Mat overlayStereoImagePair(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        cv::Mat overlay;
        cv::addWeighted(left_image, 0.5, right_image, 0.5, 0, overlay);

        cv::Mat blue_mask, red_mask;
        cv::cvtColor(left_image, blue_mask, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_image, red_mask, cv::COLOR_BGR2GRAY);
        cv::Mat blue_overlay, red_overlay;
        cv::applyColorMap(blue_mask, blue_overlay, cv::COLORMAP_COOL);
        cv::applyColorMap(red_mask, red_overlay, cv::COLORMAP_HOT);

        cv::Mat combined_overlay;
        cv::addWeighted(blue_overlay, 0.5, red_overlay, 0.5, 0, combined_overlay);

        return combined_overlay;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_publisher_;
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
    bool vis_raw_cam_img_;
    bool vis_rect_img_overlay_;
    bool publish_raw_images_;
    bool publish_rectified_images_;
    bool minimize_motion_blur_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;
    cv::Mat rectified_left_, rectified_right_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}