#include "pathfinder.h"


pathfinder::pathfinder(const nav_msgs::msg::OccupancyGrid &map) : Node("pathfinder"), CELL_OCCUPIED_THRESHOLD(75), CELL_VALUE_MIN(-1), CELL_VALUE_MAX(100), map_(map)
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_visualization", 10);
}

void pathfinder::publishMarkers(const geometry_msgs::msg::PoseArray &path)
{
    visualization_msgs::msg::MarkerArray markerArray;

    for (unsigned int i = 0; i < path.poses.size(); i++)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "path";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = path.poses.at(i);
        marker.lifetime = rclcpp::Duration::from_seconds(1);    // this may need to be changed

        // start marker
        if (i == 0)
        {
            // scale
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

        // end marker 
        }

        else if (i == path.poses.size() - 1) 
        {
            // scale
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

        // any other marker
        }

        else
        {
            // scale
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        
        markerArray.markers.push_back(marker);
    }

    marker_pub_->publish(markerArray);
}

void pathfinder::updateMap(const nav_msgs::msg::OccupancyGrid &msg)
{
    std::unique_lock<std::mutex> lck(mtx_);
    map_ = msg;
}

nav_msgs::msg::OccupancyGrid pathfinder::getMap(void)
{
    std::unique_lock<std::mutex> lck(mtx_);
    return map_;
}

int pathfinder::cart2cell(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map)
{
    // Convert to grid coordinates
    int column = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
    int row = (pose.position.y - map.info.origin.position.y) / map.info.resolution;

    // bounds checking
    if (column < 0 || column >= map.info.width || row < 0 || row >= map.info.height)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Pose (" << pose.position.x << ", " << pose.position.y << ") is out of bounds");
        return -1;
    }

    // find index
    int index = (row * map.info.width) + column;

    RCLCPP_INFO_STREAM(this->get_logger(), "Pose: (" << pose.position.x << ", " << pose.position.y << ") | Cell: [" << column << ", " << row << "] | Index: " << index);

    return index;


    /*
    THIS IS HERE AS BACKUP BUT I THINK MIGHT BE OVERKILL CURRENTLY
    // Transform the pose from the world frame to the map frame
    tf2::Quaternion mapQuat;
    tf2::fromMsg(map.info.origin.orientation, mapQuat);
    tf2::Vector3;
    tf2::Vector3 mapVec;
    tf2::fromMsg(map.info.origin.position, mapVec);
    tf2::Transform mapTF(mapQuat, mapVec);

    tf2::Quaternion poseQuat;
    tf2::fromMsg(pose.orientation, poseQuat);
    tf2::Vector3 poseVec;
    tf2::fromMsg(pose.position, poseVec);
    tf2::Transform poseTF(poseQuat, poseVec);
    
    mapTF *= poseTF;

    // Convert to cell units
    double column = mapTF.getOrigin().x() / map.info.resolution;
    double row = mapTF.getOrigin().y() / map.info.resolution;

    // Bounds checking
    if (column < 0 || column >= map.info.width || row < 0 || row >= map.info.height)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Pose (" << pose.position.x << ", " << pose.position.y << ") is out of bounds., Equates to cell [" << column << "/" << map.info.width << ", " << row << "/" << map.info.height << "]");
        return -1;
    }

    // Convert from (x,y) to row-major index
    int index = ((map.info.height - row - 1) * map.info.width) + column;

    return index;
    */
}

geometry_msgs::msg::Pose pathfinder::cell2cart(const int &index, const nav_msgs::msg::OccupancyGrid &map)
{
    geometry_msgs::msg::Pose pose;

    // Convert to cartesian coordinates ((grid_cell * resolution) + origin)
    pose.position.x = ((index % map.info.width) * map.info.resolution) + map.info.origin.position.x;
    pose.position.y = ((index / map.info.width) * map.info.resolution) + map.info.origin.position.y;

    return pose;
}

std::array<int, 8> pathfinder::findAdjacentCells(const int &index, const nav_msgs::msg::OccupancyGrid &map)
{   
    // THIS IS WHERE WE ARE UP TO IN OUR REVIEW
    // I THINK I HAVE MIXED HOW THE ROW MAJOR WORKS SO I AM GOING OVER IT ALL FROM SCRATCH AGAIN
    // (0,0) IS BOTTOM LEFT WITH X = +1 AND y = +ROW_WIDTH
    std::array<int, 8> adjacentCells = {-1, -1, -1, -1, -1, -1, -1, -1};

    // limit map size to INT_MAX cells so I dont have to keep changing variables
    if (map.info.width * map.info.height > INT_MAX)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Map too large to process, size [" << map.info.width * map.info.height << " / " << INT_MAX << "]");
        return adjacentCells;
    }

    // bound flags
    bool isLeft = false;
    bool isRight = false;

    if (index % map.info.width == 0)
    {
        // you are on the left side of the grid
        isLeft = true;
    }

    if (index % map.info.width == map.info.width - 1)
    {
        // you are on the right side of the grid
        isRight = true;
    }

    // Y+
    if (!(index < map.info.width))
    {
        adjacentCells[0] = index - map.info.width;

        // X+
        if (!isRight)
        {
            adjacentCells[1] = index - map.info.width + 1;
        }
        
        // X-
        if (!isLeft)
        {
            adjacentCells[7] = index - map.info.width - 1;
        }
    }


    // X+
    if (!isRight)
    {
        adjacentCells[2] = index + 1;
    }

    // X-
    if (!isLeft)
    {
        adjacentCells[6] = index - 1;
    }

    // Y-
    if (!(index >= map.info.height * map.info.width - map.info.width))
    {
        adjacentCells[4] = index + map.info.width;

        if (!isRight)
        {
            adjacentCells[3] = index + map.info.width + 1;
        }

        if (!isLeft)
        {
            adjacentCells[5] = index + map.info.width - 1;
        }
    }


    // set all out of bounds cells to -1
    for (int i = 0; i < 8; i++)
    {
        if (adjacentCells[i] < 0 || adjacentCells[i] >= static_cast<int>(map.info.width * map.info.height))
        {
            adjacentCells[i] = -1;
        }
    }

    return adjacentCells;
}

double pathfinder::findEuklidDistance(const unsigned int &startCell, const unsigned int &goalCell, const nav_msgs::msg::OccupancyGrid &map)
{
    // X and Y distances
    int startColumn = startCell % map.info.width;
    int goalColumn = goalCell % map.info.width;
    int xDist = std::abs(startColumn - goalColumn);

    int startRow = startCell / map.info.width;
    int goalRow = goalCell / map.info.width;
    int yDist = std::abs(startRow - goalRow);

    // find diagonals
    unsigned int diagonalMoves = std::min(xDist, yDist);
    double cost = diagonalMoves * sqrt(2);

    // find linear moves
    cost += std::max(xDist, yDist) - diagonalMoves;

    return cost;
}

void pathfinder::reorderQueue(std::priority_queue<cellData, std::vector<cellData>, Compare> &queue) {
    std::priority_queue<cellData, std::vector<cellData>, Compare> tempList;
    tempList.swap(queue);

    while (!tempList.empty())
    {
        queue.push(tempList.top());
        tempList.pop();
    }
}

geometry_msgs::msg::PoseArray pathfinder::AStar(const geometry_msgs::msg::Pose &startPose, const geometry_msgs::msg::Pose &goalPose)
{
    nav_msgs::msg::OccupancyGrid map = getMap();
    geometry_msgs::msg::PoseArray path;

    // get start and end cells
    unsigned int startCell = cart2cell(startPose, map);
    unsigned int goalCell = cart2cell(goalPose, map);

    // check if start and goal poses are valid
    if (startCell == -1)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start cell is invalid");
        return path;
    }

    if (startCell == -1)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "End cell is invalid");
        return path;
    }

    if (static_cast<int>(map.data[startCell]) > CELL_OCCUPIED_THRESHOLD)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start cell is occupied");
        return path;
    }

    if (static_cast<int>(map.data[goalCell]) > CELL_OCCUPIED_THRESHOLD)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "goal cell is occupied");
        return path;
    }

    // check if start and goal cells are the same
    if (startCell == goalCell)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start and goal poses are in the same cell");
        path.poses.push_back(goalPose);
        return path;
    }

    // generate new cell data
    std::vector<cellData> cellDataVector(map.data.size());
    for (unsigned int i = 0; i < map.data.size(); i++)
    {
        cellDataVector.emplace_back
                                (
                                i,                                  // index of cell
                                false,                              // has this cell been visited (g, h and f cost calculated)
                                static_cast<int>(map.data.at(i)),   // occupancy value
                                UINT_MAX,                           // index of parent cell
                                INFINITY,                           // (cost to get there from start)
                                INFINITY,                           // (cost to get to goal from here)
                                INFINITY                            // (G cost + H cost)
                                );
    }

    // set start cell data
    cellDataVector.at(startCell) = {cellDataVector.at(startCell).index,
                                    true,
                                    cellDataVector.at(startCell).occupancy,
                                    startCell,
                                    0,
                                    0,
                                    0
                                    };

    // list
    std::priority_queue<cellData, std::vector<cellData>, Compare> openList;
    openList.push(cellDataVector.at(startCell));
    std::vector<int> closedList(cellDataVector.size());

    bool goalFound = false;

    while (!openList.empty())
    {
        // new cycle
        cellData currentCell = openList.top();
        openList.pop();
        closedList.push_back(currentCell.index);
        bool updatedList = false;

        // look at surrounding cells
        std::array<int, 8> adjacentCells = findAdjacentCells(currentCell.index, map);
        for (int i = 0; i < 8; i++)
        {
            // validity check
            if (adjacentCells[i] == -1)
            {
                continue;
            }

            // check if goal
            if (static_cast<unsigned int>(adjacentCells[i]) == goalCell)
            {
                goalFound = true;
            }

            // check if cell is in closed list
            if (std::find(closedList.begin(), closedList.end(), adjacentCells[i]) != closedList.end())
            {
                continue;
            }

            // check if cell occupied
            if (cellDataVector.at(adjacentCells[i]).occupancy >= CELL_OCCUPIED_THRESHOLD)
            {
                closedList.push_back(adjacentCells[i]);
                continue;
            }

            // set costs
            if (cellDataVector.at(adjacentCells[i]).hCost == INFINITY)
            {
                cellDataVector.at(adjacentCells[i]).hCost = findEuklidDistance(adjacentCells[i], goalCell, map);
            }

            double newGCost = findEuklidDistance(currentCell.index, adjacentCells[i], map);
            newGCost += currentCell.gCost;
            double newFCost = cellDataVector.at(adjacentCells[i]).hCost + newGCost;
            
            // make current cell new parent if new cost is lower
            if (newFCost < cellDataVector.at(adjacentCells[i]).fCost)
            {
                // update cell data
                cellDataVector.at(adjacentCells[i]).parentCell = currentCell.index;
                cellDataVector.at(adjacentCells[i]).gCost = newGCost;
                cellDataVector.at(adjacentCells[i]).fCost = newFCost;

                // flag for list update
                if (cellDataVector.at(adjacentCells[i]).visited)
                {
                    updatedList = true;
                }
            }

            // add to open list if not already visited
            if (!cellDataVector.at(adjacentCells[i]).visited)
            {
                cellDataVector.at(adjacentCells[i]).visited = true;
                openList.push(cellDataVector.at(adjacentCells[i]));
            }
        }

        // Recalculate list if data updated
        if (updatedList)
        {
            reorderQueue(openList);
        }
    
        // if goal found, clear open list to end loop
        if (goalFound)
        {
            // Clear the queue
            std::priority_queue<cellData, std::vector<cellData>, Compare>().swap(openList);
        }
    }

    // check if goal found
    if (!goalFound)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Cannot find path");
        return path;
    }

    // generate path (this is generating backwards)
    cellData currentCell = cellDataVector.at(goalCell);
    geometry_msgs::msg::Pose pose = cell2cart(goalCell, map);

    while (currentCell.parentCell != startCell)
    {
        // yaw is angle from parent
        geometry_msgs::msg::Pose parentPose = cell2cart(currentCell.parentCell, map);
        double yaw = atan2(pose.position.y - parentPose.position.y, pose.position.x - parentPose.position.x);
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        pose.orientation = tf2::toMsg(quat);

        // add pose to path
        path.poses.push_back(pose);
        pose = parentPose;
        currentCell = cellDataVector.at(currentCell.parentCell);
    }

    // reverse path
    std::reverse(path.poses.begin(), path.poses.end());

    //publishMarkers(path);

    return path;
}