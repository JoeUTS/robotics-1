#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <mutex>
#include <climits>  // for INT_MAX
#include <cmath>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

struct cellData {
        unsigned int index;         // index of cell
        bool visited;               // has this cell been visited (g, h and f cost calculated)
        int occupancy;              // occupancy value
        unsigned int parentCell;    // index of parent cell
        double gCost;               // (cost to get there from start)
        double hCost;               // (cost to get to goal from here)
        double fCost;               // (G cost + H cost)
};

class pathfinder : public rclcpp::Node {

public:
    pathfinder(const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief Get the Map
     * 
     * @return nav_msgs::msg::OccupancyGrid map_
     */
    nav_msgs::msg::OccupancyGrid getMap(void);

    /**
     * @brief ROS2 callback for map data
     * 
     * @param msg 
     */
    void updateMap(const nav_msgs::msg::OccupancyGrid &msg);

    /**
     * @brief Calculates path using A* algorithm
     * 
     * @param startPose geometry_msgs::msg::Pose
     * @param goalPose geometry_msgs::msg::Pose
     * @return geometry_msgs::msg::PoseArray path waypoints
     */
    geometry_msgs::msg::PoseArray AStar(const geometry_msgs::msg::Pose startPose, const geometry_msgs::msg::Pose goalPose);

private:
    std::mutex mtx_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; //!< Marker publisher

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                //!< TF buffer
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   //!< Transform listener

    const int CELL_OCCUPIED_THRESHOLD;  //!< Cell occupied threshold value
    const int CELL_VALUE_MIN;           //!< Cell min value
    const int CELL_VALUE_MAX;           //!< Cell max value

    nav_msgs::msg::OccupancyGrid map_;    //!< Map data

    /**
     * @brief publish's path markers
     * 
     * @param path 
     */
    void publishMarkers(const geometry_msgs::msg::PoseArray path);

    /**
     * @brief convert point to map coords
     * 
     * @param pose 
     * @param map 
     * @return int cell index
     */
    int cart2cell(const geometry_msgs::msg::Pose pose, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief convert cell index to map coords
     * 
     * @param index 
     * @param map 
     * @return geometry_msgs::msg::Pose center point of cell
     */
    geometry_msgs::msg::Pose cell2cart(const int index, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief find the adjacent cell ID's in clockwise order starting from the positive Y axis.
     * 
     * @param index current cell
     * @param map 
     * @return std::array<int, 8> ID of adjacent cells. -1 represents no cell
     */
    std::array<int, 8> findAdjacentCells(const int index, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief Finds distance between cells using modified manhattan distance. \n
     *        Only allowing 8 directional movement between cell centers for simplicity. \n
     *        horizontal and vertical movement = cost 1. \n 
     *        diagonal movement = cost squrt(2).
     * @param startCell 
     * @param goalCell 
     * @param map 
     * @return double distance between cells as number of cells.
     */
    double findEuklidDistance(const unsigned int startCell, const unsigned int goalCell, const nav_msgs::msg::OccupancyGrid &map);
};

#endif // PATHFINDER_H