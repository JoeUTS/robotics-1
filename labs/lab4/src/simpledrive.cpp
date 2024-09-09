#include "simpledrive.h"

simpledrive::simpledrive() : Node("simpledrive"), GOAL_DISTANCE(1), NOISE_MAX(0.1) {
    totalDistance_ = 0;
    vel_.linear.x = 1;  // 1m/s
    
    // initialising flags
    prevOdoSet_ = 0;

    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&simpledrive::odoCallback,this,std::placeholders::_1));

    pub_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pub_noisey_odo_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/noisey_odo", 10);
    
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&simpledrive::timerCallback, this));
}

void simpledrive::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    odo_ = *msg;
}

void simpledrive::timerCallback(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    // populate previous odometry values first
    if (!prevOdoSet_) {
        prevOdo_ = odo_;
        lastTime_ = this->now();
        pub_cmd_vel_->publish(vel_);    // start driving
        prevOdoSet_ = 1;
        return;
    }

    if (totalDistance_ >= GOAL_DISTANCE) {
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist()); // stop driving
            RCLCPP_INFO_STREAM(this->get_logger(), "We have arrived at our destination");
        return;
    }

    // copy member variables
    nav_msgs::msg::Odometry localOdo = odo_;
    nav_msgs::msg::Odometry localLastOdo = prevOdo_;
    geometry_msgs::msg::Twist localvel = vel_;
    rclcpp::Time localLastTime = lastTime_;
    lck.unlock();

    // current odometry
    double localX = localOdo.pose.pose.position.x;
    double localY = localOdo.pose.pose.position.y;
    double localRoll;
    double localPitch;
    double localYaw;
    tf2::Quaternion quatTF2;
    tf2::fromMsg(localOdo.pose.pose.orientation, quatTF2);
    tf2::Matrix3x3(quatTF2).getRPY(localRoll, localPitch, localYaw);
    
    // get velocities
    double velLin = localvel.linear.x;
    double velYaw = localvel.angular.z;

    // calculate delta time
    rclcpp::Time currentTime = this->now();
    double duration = (currentTime - localLastTime).seconds();

    // calculate pose change
    double distanceTraveled = velLin * duration;
    double deltaYaw = velYaw * duration;



    // break into individual components
    double deltaX = distanceTraveled * cos(deltaYaw);
    double deltaY = distanceTraveled * sin(deltaYaw);

    // update local pose with noise
    localX += deltaX + getNoise(-NOISE_MAX, NOISE_MAX);
    localY += deltaY + getNoise(-NOISE_MAX, NOISE_MAX);
    localYaw += deltaYaw + getNoise(-NOISE_MAX, NOISE_MAX);

    // Yaw correction
    if (localYaw > M_PI) {
        localYaw -= 2 * M_PI;
    } else if (localYaw < -M_PI) {
        localYaw += 2 * M_PI;
    }

    // Generate noisey message
    localOdo.header.stamp = this->now();
    localOdo.pose.pose.position.x = localX;
    localOdo.pose.pose.position.y = localY;
    quatTF2.setRPY(0.0, 0.0, localYaw);
    localOdo.pose.pose.orientation = tf2::toMsg(quatTF2);
    localOdo.twist.twist = localvel;

    // update member variables
    lck.lock();
    totalDistance_ += distanceTraveled;
    prevOdo_ = odo_;
    lastTime_ = this->now();
    lck.unlock();

    pub_noisey_odo_->publish(localOdo);
}

double simpledrive::getNoise(const double distLower, const double distUpper) {
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(distLower,distUpper);
    double output = distribution(generator);

    return output;
}