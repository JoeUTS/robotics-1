#include "slam.h"


slam::slam() : Node("slam") {

    // timer
    const int fps = 60;
    const int dt = 1000 / fps;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(dt),
        std::bind(&slam::timerCallback, this));
}

void slam::timerCallback(void) {
    
}