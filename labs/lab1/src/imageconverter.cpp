#include "imageconverter.h"

image_converter::image_converter() : Node("image_converter") {
    subImage_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&image_converter::imageCallback,this,std::placeholders::_1));

    pubImage_ =
        this->create_publisher<sensor_msgs::msg::Image>("/modified_image", 10);
}

void image_converter::imageCallback(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "rgb8")->image;
        //cv::imshow("camera", frame);
        //cv::waitKey(10);

        if (frame.rows > 60 && frame.cols > 60) {
            cv::circle(frame, cv::Point(frame.size().width/2, frame.size().height/2), 10, CV_RGB(255,0,0));
            //cv::imshow("camera", frame);
            //cv::waitKey(10);

            std::shared_ptr<sensor_msgs::msg::Image> responce = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
            
            pubImage_->publish(*responce);
        }

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    

}