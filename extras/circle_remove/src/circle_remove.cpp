#include "circle_remove.h"

circle_remove::circle_remove() : Node("circle_remove") {
    // map
    mapSet_ = false;

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&circle_remove::mapCallback, this, std::placeholders::_1));

    // odometry
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&circle_remove::odoCallback,this,std::placeholders::_1));

    // laserscan
    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&circle_remove::scanCallback, this, std::placeholders::_1));

    // goals
    goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/goalPose", 10, std::bind(&circle_remove::goalCallback,this,std::placeholders::_1));
    
    // navigation
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // timer
    const int fps = 60;
    const int dt = 1000 / fps;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(dt),
        std::bind(&circle_remove::timerCallback, this));
}

void circle_remove::mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg) {
    map_ = *msg;
    mapImage_ = mapToMat(map_);
    mapSet_ = true;
}

void circle_remove::scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    laserScan_ = *msg;

    // add found objects to list
    if (objects_.size() < 1) {
        // set object
        objects_ = objectDetect(*msg);
    } else {
        std::vector<cv::Point> foundObjects = objectDetect(*msg);
        
        double tollerance = 0.1;    // [m]

        for (cv::Point newObject : foundObjects) { 
            // filter existing objects
            for (cv::Point oldObject : objects_) {
                if (std::hypot(newObject.x - oldObject.x, newObject.y - oldObject.y) > tollerance) {
                    // add to list if greater than tollerance
                    objects_.push_back(newObject);
                } 
            }
        }
    }
}

void circle_remove::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    odo_ = *msg;
    // add an odo set
}

void circle_remove::goalCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) {
    goalPose_ = *msg;
    goalSet_ = true;
}

void circle_remove::timerCallback(void) {
    if (mapSet_) {
        showImages();
    }
}

void circle_remove::sendGoal(const geometry_msgs::msg::Pose &goal) {
    // Wait for the action server to be available
    while (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    nav2_msgs::action::NavigateToPose::Goal goalMsg;
    goalMsg.pose.header.frame_id = "map";
    goalMsg.pose.header.stamp = this->now();
    goalMsg.pose.pose.position.x = goal.position.x;
    goalMsg.pose.pose.position.y = goal.position.y;
    goalMsg.pose.pose.orientation.w = goal.orientation.w;
    goalMsg.pose.pose.orientation.x = goal.orientation.x;
    goalMsg.pose.pose.orientation.y = goal.orientation.y;
    goalMsg.pose.pose.orientation.z = goal.orientation.z;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
            //RCLCPP_INFO(this->get_logger(), "Goal accepted!");
      };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            //RCLCPP_INFO(this->get_logger(), "Goal result received!");
        };

    auto goal_handle = nav_client_->async_send_goal(goalMsg, send_goal_options);
}

cv::Mat circle_remove::mapToMat(const nav_msgs::msg::OccupancyGrid &map) {
    // make image
    int imageWidth = map.info.width;
    int imageHeight = map.info.height;

    cv::Mat image(imageHeight, imageWidth, CV_8UC1, cv::Scalar(0));

    // Populate the image with data from the occupancy grid
    for (int i = imageHeight - 1; i >= 0; i--) {
        for (int j = 0; j < imageWidth; j++) {
            int index = i * imageWidth + j;
            if (map.data[index] == -1) {  // unknown
                image.at<uchar>(imageHeight - 1 - i, j) = 128;
            } else if (map.data[index] == 0) {  // free
                image.at<uchar>(imageHeight - 1 - i, j) = 255;
            } else if (map.data[index] == 100) {  // occupied
                image.at<uchar>(imageHeight - 1 - i, j) = 0;
            }
        }
    }

    return image;
}

std::vector<cv::Point> circle_remove::laser2Points(const sensor_msgs::msg::LaserScan &laserScan) {
    // convert to points | (0,0) = invalid / nil result
    std::vector<cv::Point> laserReturns(laserScan.ranges.size());
    for (unsigned int i = 0; i < laserScan.ranges.size(); i++) {
        // filter invalid results
        if (laserScan.ranges.at(i) < laserScan.range_min || laserScan.ranges.at(i) > laserScan.range_max || isnan(laserScan.ranges.at(i)) || isinf(laserScan.ranges.at(i))) {
            laserReturns.at(i) = cv::Point(0, 0);
            continue;
        }

        // calculate angle
        double angle = (i * laserScan.angle_increment) - laserScan.angle_min;

        laserReturns.at(i) =  laserScan.ranges.at(i) * cv::Point(cos(angle), sin(angle));
    }

    return laserReturns;
}

cv::Point circle_remove::findCenter(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3) {
    int x12 = p1.x - p2.x;
    int x13 = p1.x - p3.x;

    int y12 = p1.y - p2.y;
    int y13 = p1.y - p3.y;

    int y31 = p3.y - p1.y;
    int y21 = p2.y - p1.y;

    int x31 = p3.x - p1.x;
    int x21 = p2.x - p1.x;

    // x1^2 - x3^2
    int sx13 = pow(p1.x, 2) - pow(p3.x, 2);

    // y1^2 - y3^2
    int sy13 = pow(p1.y, 2) - pow(p3.y, 2);

    int sx21 = pow(p2.x, 2) - pow(p1.x, 2);
    int sy21 = pow(p2.y, 2) - pow(p1.y, 2);

    int f = ((sx13) * (x12)
             + (sy13) * (x12)
             + (sx21) * (x13)
             + (sy21) * (x13))
            / (2 * ((y31) * (x12) - (y21) * (x13)));
    int g = ((sx13) * (y12)
             + (sy13) * (y12)
             + (sx21) * (y13)
             + (sy21) * (y13))
            / (2 * ((x31) * (y12) - (x21) * (y13)));

    int c = -pow(p1.x, 2) - pow(p1.y, 2) - 2 * g * p1.x - 2 * f * p1.y;

    int radius = sqrt(-g * -g + -f * -f - c);   /// not used currently

    return cv::Point(-g, -f);
}

std::vector<cv::Point> circle_remove::objectDetect(const sensor_msgs::msg::LaserScan &laserScan) {
    const double diameter = 0.3;       // [m]
    const double diameterError = 0.1;  // [m]

    std::vector<cv::Point> laserReturns = laser2Points(laserScan);

    // find objects
    std::vector<cv::Point> objects;     // holds object centers
    bool inObject = false;              // represents if currently examining an object
    unsigned int startIndex = UINT_MAX; // UINT_MAX represents no start index
    double newObjectThesh = 0.075;      // [m] distance between returns to consider new.

    for (unsigned int i = 0; i < laserReturns.size(); i++) {
        cv::Point currentPoint = laserReturns.at(i);

        if (currentPoint.x == 0 && currentPoint.y == 0) {
            // nothing
            if (inObject) {
                // end object
                RCLCPP_ERROR_STREAM(this->get_logger(), "Ending Object");
                
                // find diameter
                double distance = std::hypot(laserReturns.at(startIndex).x - laserReturns.at(i - 1).x, laserReturns.at(startIndex).y - laserReturns.at(i - 1).y);

                // check if object is big enough
                if (std::abs(distance - diameter) < diameterError) {
                    // we have found the right sized object!
                    // find center
                    int midIndex = startIndex + ((i - startIndex) / 2);
                    cv::Point center = findCenter(laserReturns.at(startIndex), laserReturns.at(i - 1), laserReturns.at(midIndex));
                    
                    // convert to world frame
                    center.x += odo_.pose.pose.position.x;
                    center.y += odo_.pose.pose.position.y;
                    objects.emplace_back(center);
                }  else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "wrong size object: " << distance);
                }

                inObject = false;
                startIndex = UINT_MAX;
            }

        } else if (!inObject) {
            // start object
            RCLCPP_ERROR_STREAM(this->get_logger(), "new Object");
            startIndex = i;
            inObject = true;
        } else {
            // check if same object
            double distance = std::hypot(laserReturns.at(i - 1).x - laserReturns.at(i).x, laserReturns.at(i - 1).y - laserReturns.at(i).y);

            if (distance > newObjectThesh) {
                // new object
                RCLCPP_ERROR_STREAM(this->get_logger(), "Ending Object");

                // find diameter
                double distance = std::hypot(laserReturns.at(startIndex).x - laserReturns.at(i - 1).x, laserReturns.at(startIndex).y - laserReturns.at(i - 1).y);

                // check if object is big enough
                if (std::abs(distance - diameter) < diameterError) {
                    // we have found the right sized object!!!
                    // find center
                    int midIndex = startIndex + ((i - startIndex) / 2);
                    cv::Point center = findCenter(laserReturns.at(startIndex), laserReturns.at(i - 1), laserReturns.at(midIndex));
                    
                    // convert to world frame
                    center.x += odo_.pose.pose.position.x;
                    center.y += odo_.pose.pose.position.y;
                    objects.emplace_back(center);
                } else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "wrong size object: " << distance);
                }

                RCLCPP_ERROR_STREAM(this->get_logger(), "new Object");
                startIndex = i;
            }
        }
    }

    // needs fix for wrap around issues

    return objects;
}

cv::Point circle_remove::worldPoseToMap(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map) {
    cv::Point mapPoint;
    mapPoint.x = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
    mapPoint.y = map.info.height - (pose.position.y - map.info.origin.position.y) / map.info.resolution;

    return mapPoint;
}

void circle_remove::showImages(void) {
    // set up image
    cv::Mat mainImage;
    cv::cvtColor(mapImage_, mainImage, cv::COLOR_GRAY2BGR);

    // circle finding
    for (const cv::Point &center : objects_) {
        cv::circle( mainImage,                           // image to draw on
                    center,                              // circle center
                    5,                                   // circle radius (WE NEED TO FIGURE THIS ONE OUT!!!)
                    cv::Scalar(0, 0, 255),               // colour
                    1);                                  // line thickness (-1 is filled)

        cv::circle( mainImage,                           // image to draw on
                    center,     // circle center
                    2,                                   // circle radius
                    cv::Scalar(255, 0, 0),               // colour
                    -1);                                 // line thickness (-1 is filled)
    }

    // path finding
    if (goalSet_) {
        sendGoal(goalPose_);

        cv::Point goalPoint = worldPoseToMap(goalPose_, map_);
        
        cv::circle( mainImage,              // image to draw on
                    goalPoint,          // circle center
                    6,                      // circle radius
                    cv::Scalar(0, 0, 255),  // colour
                    1);                     // line thickness (-1 is filled)
    }

    // robot location
    cv::Point robotLocation = worldPoseToMap(odo_.pose.pose, map_);
    
    cv::circle( mainImage,              // image to draw on
                robotLocation,          // circle center
                2,                      // circle radius
                cv::Scalar(255, 0, 0),  // colour
                -1);                    // line thickness (-1 is filled)
    
    cv::Mat newImage;
    cv::resize(mainImage, newImage, cv::Size(0, 0), 2, 2, cv::INTER_NEAREST);
    cv::imshow("image", newImage);
    cv::waitKey(1);
    
}

