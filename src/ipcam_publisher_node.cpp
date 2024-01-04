#include <cstdio>
#include "ros2_ipcam_publisher/ipcam_publisher_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

IPCamPublisherNode::IPCamPublisherNode(const rclcpp::NodeOptions & options)
  : Node("ipcam_publisher_node", options){

  publishing_frames = false;

  // Get ROS params
  this->declare_parameter("camera_ip", std::string(""));
  this->declare_parameter<std::vector<int64_t>>("image_size", {640, 480});
  this->declare_parameter<std::int64_t>("frame_rate", 100);

  cam_ip = this->get_parameter("camera_ip").as_string();
  this->get_parameter("image_size", image_size_);
  this->get_parameter("frame_rate", frame_rate);

  RCLCPP_INFO(this->get_logger(), "Camera IP: %s", cam_ip.c_str());
  RCLCPP_INFO(this->get_logger(), "image size: %d , %d", image_size_[0], image_size_[1]);
  RCLCPP_INFO(this->get_logger(), "frame_rate: %d", frame_rate);

  // initialize the ip camera
  this->initialize_ipcam();

  // Create publisher for sensor_msgs/Image
  auto qos_profile = rclcpp::SensorDataQoS().reliable().durability_volatile();
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile);

  // Create timer to publish video frames at a fixed rate
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frame_rate), std::bind(&IPCamPublisherNode::publishFrame, this));
}


void IPCamPublisherNode::initialize_ipcam(){
  // Open the video capture
  cap = cv::VideoCapture("http://" + cam_ip + ":8080/video");
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening video stream from the device ip: %s", cam_ip.c_str());
    // rclcpp::shutdown();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Attempt to reconnect ...");
    cap.release();
    this->initialize_ipcam();
  }

  // Set video capture size
  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_size_[0]);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_size_[1]);
}


void IPCamPublisherNode::publishFrame(){
  cv::Mat frame;
  cap.read(frame);

  if (!publishing_frames && !frame.empty()){
    RCLCPP_INFO(this->get_logger(), "Camera is connected successfully. publishing image frames ...");
    publishing_frames = true;
  }

  if(!frame.empty()){
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->height = frame.rows;
    msg->width = frame.cols;
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(frame.step);
    size_t size = frame.total() * frame.elemSize();
    msg->data = std::vector<uint8_t>(frame.data, frame.data + size);

    publisher_->publish(std::move(msg));
  }
  else{
    publishing_frames = false;
    RCLCPP_ERROR(this->get_logger(), "Camera is disconnected. Frame is empty!");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Attempt to reconnect ...");
    cap.release();
    this->initialize_ipcam();
  }
}



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  // printf("hello world ros2_smartphone_cam_publisher package\n");

  rclcpp::init(argc,  argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<IPCamPublisherNode>(options));
  rclcpp::shutdown();
  return 0;
}
