#ifndef _VM_HPP_
#define _VM_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "line_tracer/dxl.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <queue>
#include <ctime>
#include <unistd.h>
#include <signal.h>
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace cv;
class Sub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
        void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;

        Mat frame, ROI, gray, correct, bin, morpology, labels, stats, centroids, colorimg;
        int cnt;
        int vel1 = 0,vel2 = 0;
        double ynum=0;
        double error = 0, realline = 0;
        bool first = true;

        double center_x = 320;

        VideoWriter writer1;
        VideoWriter writer2;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        std_msgs::msg::Int32 intmsg;
        void publish_msg();
    public:
        Sub();
        int err;
};
#endif //_SUB_HPP_
