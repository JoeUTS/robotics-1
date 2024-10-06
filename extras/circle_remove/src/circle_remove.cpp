#include "circle_remove.h"

circle_remove::circle_remove() : Node("circle_remove") {
    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&circle_remove::scanCallback, this, std::placeholders::_1));
}

void circle_remove::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laserScan_ = *msg;

    showImages();
}

cv::Mat circle_remove::laserScanToMat(const sensor_msgs::msg::LaserScan &laserScan) {
    // Define the image size and resolution
    int imageWidth = 512;
    int imageHeight = 512;
    double resolution = 0.01; // meters per pixel

    // Create the image
    cv::Mat image(imageHeight, imageWidth, CV_8UC1, cv::Scalar(0));
    // maybe this will make it work
    std::vector<geometry_msgs::msg::Point> points;
    // convert to points
    // find distnance between points
    // draw line between points if below threshold distance

    // Iterate over the range values
    for (int i = 0; i < laserScan.ranges.size(); i++) {
        // Calculate the Cartesian coordinates
        double angle = laserScan.angle_min + i * laserScan.angle_increment;
        double range = laserScan.ranges[i];
        double x = range * cos(angle);
        double y = range * sin(angle);

        // Convert to image coordinates
        int pixelX = (int)((x / resolution) + imageWidth / 2);
        int pixelY = (int)((y / resolution) + imageHeight / 2);

        // Assign the pixel value
        if (pixelX >= 0 && pixelX < imageWidth && pixelY >= 0 && pixelY < imageHeight) {
        image.at<uchar>(pixelY, pixelX) = (uchar)(255 * (1 - range / laserScan.range_max));
        }
    }

    return image;
}

void circle_remove::showImages(void) {
    cv::Mat image1 = laserScanToMat(laserScan_);

    // make image copies for display
    cv::Mat image2 = image1.clone();
    cv::Mat image3 = image1.clone();

    /*
    Find circles in image
    structure of Vec3f
    Vec3f[0]: x
    Vec3f[1]: y
    Vec3f[2]: radius
    */
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(image1,                // input image
                     circles,               // output container
                     cv::HOUGH_GRADIENT,    // method
                     1,                     // resolution
                     10,                    // min dist betwen circles centers
                     100,                   // edge detection upper thresh
                     30,                    // dection threshold
                     1,                     // min radius
                     20);                   // max radius

    for (const cv::Vec3f& circle : circles) {
        if (circle[2] > 0.1 && circle[2] < 20) {
            // Draw the circle on image 2
            cv::circle(image2,                              // image to draw on
                       cv::Point(circle[0], circle[1]),    // circle center
                       circle[2],                           // circle radius
                       cv::Scalar(0, 255, 0),               // colour
                       -1);                                 // line thickness (-1 is filled)

            // "draw" the circle on image 3
            cv::circle(image3,
                       cv::Point(circle[0], circle[1]),
                       circle[2],
                       cv::Scalar(0, 0, 0),
                       2);
        }
    }
    
    cv::imshow("Image1", image1);
    cv::imshow("Image2", image2);
    cv::imshow("Image3", image3);
    cv::waitKey(1);
}

