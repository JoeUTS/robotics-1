#include "HazardCostMap.h"

// WHAT IS THIS?
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

HazardCostMap::HazardCostMap() : lastMin_x_(-std::numeric_limits<float>::max()), lastMin_y_(-std::numeric_limits<float>::max()),
                                 lastMax_x_(std::numeric_limits<float>::max()), lastMax_y_(std::numeric_limits<float>::max())
{

}

void HazardCostMap::onInitialize(void) {
    auto node = node_.lock(); 
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    needRecalculation_ = false;
    current_ = true;
}

void HazardCostMap::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (needRecalculation_) {
    // update previous values
    lastMin_x_ = *min_x;
    lastMin_y_ = *min_y;
    lastMax_x_ = *max_x;
    lastMax_y_ = *max_y;

    // Reset the values
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    needRecalculation_ = false;

  } else {
    // temp values
    double tmp_min_x = lastMin_x_;
    double tmp_min_y = lastMin_y_;
    double tmp_max_x = lastMax_x_;
    double tmp_max_y = lastMax_y_;

    // update previous values
    lastMin_x_ = *min_x;
    lastMin_y_ = *min_y;
    lastMax_x_ = *max_x;
    lastMax_y_ = *max_y;

    // update values
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void HazardCostMap::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  
}