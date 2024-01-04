#ifndef __IPCAM_PUBLISHER_H__
#define __IPCAM_PUBLISHER_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

class IPCamPublisherNode : public rclcpp::Node {
public:
    explicit IPCamPublisherNode(const rclcpp::NodeOptions & options);

private:
    void initialize_ipcam();
    void publishFrame();

    cv::VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string cam_ip;
    std::vector<int64_t> image_size_;
    std::int64_t frame_rate;

    bool publishing_frames;
};

#endif // __IPCAM_PUBLISHER_H_