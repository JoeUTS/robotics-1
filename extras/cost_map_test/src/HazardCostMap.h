#ifndef HAZARD_COSTMAP_H
#define HAZARD_COSTMAP_H

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/parameter_events_filter.hpp>


class HazardCostMap : public nav2_costmap_2d::Layer {
public:
    HazardCostMap(void);

    /**
     * @brief initilise
     * 
     */
    void onInitialize(void);

    /**
     * @brief Finds where needs to be updated, will output rectangular area defined by min_x, min_y, max_x, max_y.
     * 
     * @param robot_x       x position of the robot in the HazardCostMap frame
     * @param robot_y       y position of the robot in the HazardCostMap frame
     * @param robot_yaw     yaw angle of the robot in the HazardCostMap frame
     * @param min_x         lower X bound of costmap update
     * @param min_y         lower Y bound of costmap update
     * @param max_x         upper X bound of costmap update
     * @param max_y         upper Y bound of costmap update
     */
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y);
    
    /**
     * @brief updates costmap. For performance will only update within bounds as defined by min_i, min_j, max_i, max_j
     * 
     * @param master_grid   Costmap to update
     * @param min_i         lower X bound of costmap update?
     * @param min_j         lower Y bound of costmap update?
     * @param max_i         upper X bound of costmap update?
     * @param max_j         upper Y bound of costmap update?
     */
    virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief called when mapsize changed
     * 
     */
    virtual void matchSize(void);

    virtual void reset(void);

private:
    double lastMin_x_, lastMin_y_, lastMax_x_, lastMax_y_;  //!< holds last update bounds

    bool needRecalculation_; //!< true if costmap needs to be updated
};

#endif // HAZARD_COSTMAP_H