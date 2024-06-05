#include "line_tracer/vm.hpp"
void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    frame = imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    //gettimeofday(&start, NULL);
	if (frame.empty()) { std::cerr << "frame empty!" << std::endl; return; }
	ROI = frame(Rect(0, 270, 640, 90));
	cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
	
	correct = gray + (cv::Scalar(100) - mean(gray));
	cv::adaptiveThreshold(correct, bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY, 141, -21);
	
	morphologyEx(bin, morpology, MORPH_CLOSE, Mat(), Point(-1, -1), 4);

	cnt = cv::connectedComponentsWithStats(morpology, labels, stats, centroids);
	cv::cvtColor(morpology, colorimg, cv::COLOR_GRAY2BGR);

	double firstdist = 100, dist = 100;

	for (int i = 1; i < cnt; i++) {
		if (stats.at<int>(i, 4) < 200)continue;

		if (first) {
			if (abs(center_x - centroids.at<double>(i, 0)) < firstdist) {
				firstdist = abs(center_x - centroids.at<double>(i, 0));
			}
			if (abs(center_x - centroids.at<double>(i, 0)) == firstdist) {
				error = center_x - centroids.at<double>(i, 0);
				realline = centroids.at<double>(i, 0);

				rectangle(colorimg, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
					stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(0, 0, 255), 2);
				circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
					1, Scalar(0, 0, 255), 2);
			}
			else {
				rectangle(colorimg, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
					stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(255, 0, 0), 2);
				circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
					1, Scalar(255, 0, 0), 2);
			}
		}
		else {
			if (abs(realline - centroids.at<double>(i, 0)) < dist) {
				dist = abs(realline - centroids.at<double>(i, 0));
			}
			if (abs(realline - centroids.at<double>(i, 0)) == dist) {
				realline = centroids.at<double>(i, 0);
				ynum=centroids.at<double>(i,1);

				rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
					stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(0, 0, 255), 2);
				circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
					1, Scalar(0, 0, 255), 2);
			}
			else {
				rectangle(colorimg, Rect(stats.at<int>(i, 0), stats.at<int>(i, 1),
					stats.at<int>(i, 2), stats.at<int>(i, 3)), Scalar(255, 0, 0), 2);
				circle(colorimg, Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)),
					1, Scalar(255, 0, 0), 2);
			}
		}
	}
	error = center_x - realline;

	if(error>265 || error<-265)circle(colorimg,Point(realline,ynum),1,Scalar(0,0,255),2);

	vel1 = 100 - 0.28* error;
	vel2 = -(100 + 0.28* error);
	if(vel1<10) vel1=10;
	if(vel2>-10) vel2=-10;
	first = false;

	writer1 << frame;
	writer2 << colorimg;
    cv::imshow("frame", frame);
    cv::imshow("color", colorimg);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
void Sub::publish_msg()
{
    intmsg.data = error;
	RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
	pub_->publish(intmsg);
}
Sub::Sub() : Node("camsub_wsl")
{
    writer1.open("output1.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(640, 360));
    writer2.open("output2.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(640, 90));
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, std::bind(&Sub::mysub_callback, this, _1));
    pub_ = this->create_publisher<std_msgs::msg::Int32>("error", qos_profile);
    timer_ = this->create_wall_timer(33ms, std::bind(&Sub::publish_msg, this));
}
  
