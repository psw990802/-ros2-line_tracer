#include "line_tracer/jetson.hpp"
Pub::Pub() : Node("campub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    timer_ = this->create_wall_timer(33ms, std::bind(&Pub::publish_msg, this));
    cap.open(src,cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return;
    }

    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        //return -1;
    } 
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("error", qos_profile,fn);
}
void Pub::publish_msg()
{
    cap >> frame;
    if (frame.empty()) { RCLCPP_ERROR(this->get_logger(), "frame empty"); return;}
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    error = intmsg->data;
    vel1 = 100 - gain * error;//왼쪽 바퀴 속도
	vel2 = -(100 + gain * error);//오른쪽 바퀴 속도
    if(vel1<10) vel1=10;
    if(vel2>-10) vel2=-10;
    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", vel1, vel2);
    mdxl.setVelocity(vel1, vel2);
}
