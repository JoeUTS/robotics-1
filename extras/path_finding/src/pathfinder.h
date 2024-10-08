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



class pathfinder : public rclcpp::Node {
public:
    pathfinder(const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief publish's path markers
     * 
     * @param path 
     */
    void publishMarkers(const geometry_msgs::msg::PoseArray &path);

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
    geometry_msgs::msg::PoseArray AStar(const geometry_msgs::msg::Pose &startPose, const geometry_msgs::msg::Pose &goalPose);

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
     * @brief cell data
     * 
     */
    struct cellData {
            unsigned int index = 0;         // index of cell
            bool visited = 0;               // has this cell been visited (g, h and f cost calculated)
            int occupancy = 0;              // occupancy value
            unsigned int parentCell = 0;    // index of parent cell
            double gCost = 0;               // (cost to get there from start)
            double hCost = 0;               // (cost to get to goal from here)
            double fCost = 0;               // (G cost + H cost)

            cellData(unsigned int index, bool visited, int occupancy, unsigned int parentCell, double gCost, double hCost, double fCost)
                : index(index), visited(visited), occupancy(occupancy), parentCell(parentCell), gCost(gCost), hCost(hCost), fCost(fCost) {}

            cellData() : index(0), visited(false), occupancy(0), parentCell(0), gCost(0), hCost(0), fCost(0) {}
    };

    /**
     * @brief Compare function for priority queue
     * 
     */
    struct Compare
    {
        bool operator()(const cellData &a, const cellData &b)
        {
            return a.fCost > b.fCost;
        }
    };

    /**
     * @brief convert point to map coords
     * 
     * @param pose 
     * @param map 
     * @return int cell index
     */
    int cart2cell(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief convert cell index to map coords
     * 
     * @param index 
     * @param map 
     * @return geometry_msgs::msg::Pose center point of cell
     */
    geometry_msgs::msg::Pose cell2cart(const int &index, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief find the adjacent cell ID's in clockwise order starting from the positive Y axis.
     * 
     * @param index current cell
     * @param map 
     * @return std::array<int, 8> ID of adjacent cells. -1 represents no cell
     */
    std::array<int, 8> findAdjacentCells(const int &index, const nav_msgs::msg::OccupancyGrid &map);

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
    double findEuklidDistance(const unsigned int &startCell, const unsigned int &goalCell, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief Reorders the priority queue
     * 
     * @param queue 
     */
    void reorderQueue(std::priority_queue<cellData, std::vector<cellData>, Compare> &queue);
};

#endif // PATHFINDER_H