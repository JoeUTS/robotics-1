#include "pathfinder.h"


pathfinder::pathfinder(const nav_msgs::msg::OccupancyGrid &map) : Node("pathfinder"), CELL_OCCUPIED_THRESHOLD(50), CELL_VALUE_MIN(-1), CELL_VALUE_MAX(100), map_(map) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_visualization", 10);
}

void pathfinder::publishMarkers(const geometry_msgs::msg::PoseArray path) {
    visualization_msgs::msg::MarkerArray markerArray;

    for (unsigned int i = 0; i < path.poses.size(); i++) {
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
        if (i == 0) {
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
        } else if (i == path.poses.size() - 1) {
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
        } else {
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

void pathfinder::updateMap(const nav_msgs::msg::OccupancyGrid &msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    map_ = msg;
}

nav_msgs::msg::OccupancyGrid pathfinder::getMap(void) {
    std::unique_lock<std::mutex> lck(mtx_);
    return map_;
}

bool pathfinder::validatePose(const geometry_msgs::msg::Pose pose, const nav_msgs::msg::OccupancyGrid &map) {
    // Only validating position currently
    int index = cart2cell(pose, map);

    // index check
    if (index < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid index @ pose[" << pose.position.x << ", " << pose.position.y << "]. Index[" << index << "]");
        return false;
    }

    int occupancy = static_cast<int>(map.data[index]);

    if (occupancy < CELL_VALUE_MIN || occupancy >= CELL_VALUE_MAX) {
        // invalid value
        // throw error
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid occupancy value @ index[" << index << "]: " << occupancy << ". expected: [" << CELL_VALUE_MIN << ", " << CELL_VALUE_MAX << "]");
        return false;
    }

    // check cell is not obstructed
    if (occupancy >= CELL_OCCUPIED_THRESHOLD) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Cell is obstructed @ index[" << index << "]: " << occupancy);
        return false;
    }

    return true;
}

int pathfinder::cart2cell(const geometry_msgs::msg::Pose pose, const nav_msgs::msg::OccupancyGrid &map) {
    // convert from global frame to map frame
    int mapX = pose.position.x - map.info.origin.position.x;
    int mapY = pose.position.y - map.info.origin.position.y;

    // change to pixle units
    mapX /= map.info.resolution;
    mapY /= map.info.resolution;

    // bounds checking
    if (mapX < 0 || mapX >= map.info.width || mapY < 0 || mapY >= map.info.height) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Pose is outside map bounds. Pose: [" << pose.position.x << ", " << pose.position.y << "], Map: [" << map.info.width << ", " << map.info.height << "]");
        return -1;
    }

    // convert from (x,y) to row-major index with (0,0) at bottom left of map
    return (mapY * map.info.width) + mapX;
}

geometry_msgs::msg::Pose pathfinder::cell2cart(const int index, const nav_msgs::msg::OccupancyGrid &map) {
    // convert to (x,y) in map frame from row-major index with (0,0) at bottom left of map
    int column = index % map.info.width;
    int row = (map.info.height - 1) - (index / map.info.width);

    // comvert to global scale
    double mapX = column * map.info.resolution;
    double mapY = row * map.info.resolution;

    // move point to center of cell
    mapX += (map.info.resolution / 2);
    mapY += (map.info.resolution / 2);

    // assign to datatype
    geometry_msgs::msg::Pose pose;
    pose.position.x = mapX;
    pose.position.y = mapY;

    return pose;
}

std::array<int, 8> pathfinder::findAdjacentCells(const int index, const nav_msgs::msg::OccupancyGrid &map) {
    std::array<int, 8> adjacentCells = {-1, -1, -1, -1, -1, -1, -1, -1};

    // limit map size to INT_MAX cells so I dont have to keep changing variables
    if (map.info.width * map.info.height > INT_MAX) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Map too large to process, size [" << map.info.width * map.info.height << " / " << INT_MAX << "]");
        return adjacentCells;
    }

    // bound flags
    bool isTop = false;
    bool isBottom = false;
    bool isLeft = false;
    bool isRight = false;

    if (index % map.info.width == 0) {
        // you are on the left side of the grid
        isLeft = true;
    }

    if (index % map.info.width == map.info.width - 1) {
        // you are on the right side of the grid
        isRight = true;
    }

    if (index - map.info.width < 0) {
        // you are on the bottom of the grid
        isBottom = true;
    }

    if (index + map.info.width > map.data.size()) {
        // you are on the top of the grid
        isTop = true;
    }

    // Y+
    if (!isTop) {
        adjacentCells[0] = index + map.info.width;

        // X+
        if (!isRight) {
            adjacentCells[1] = index + map.info.width + 1;
        }
        
        // X-
        if (!isLeft) {
            adjacentCells[7] = index + map.info.width - 1;
        }
    }


    // X+
    if (!isRight) {
        adjacentCells[2] = index + 1;
    }

    // X-
    if (!isLeft) {
        adjacentCells[6] = index - 1;
    }

    // Y-
    if (!isBottom) {
        adjacentCells[4] = index - map.info.width;

        if (!isRight) {
            adjacentCells[3] = index - map.info.width + 1;
        }

        if (!isLeft) {
            adjacentCells[5] = index - map.info.width - 1;
        }
    }


    // set all out of bounds cells to -1
    for (int i = 0; i < 8; i++) {
        if (adjacentCells[i] < 0 || adjacentCells[i] >= static_cast<int>(map.info.width * map.info.height)) {
            adjacentCells[i] = -1;
        }
    }

    return adjacentCells;
}

double pathfinder::findGridDistance(const unsigned int startCell, const unsigned int goalCell, const nav_msgs::msg::OccupancyGrid &map) {
    // find manhattan distances
    int startColumn = startCell % map.info.width;
    int goalColumn = goalCell % map.info.width;
    int xDist = std::abs(startColumn - goalColumn);

    int startRow = (map.info.height - 1) - (startCell / map.info.width);
    int goalRow = (map.info.height - 1) - (goalCell / map.info.width);
    int yDist = std::abs(startRow - goalRow);

    // Diagonal cost
    unsigned int diagonalMoves = std::min(xDist, yDist);
    double cost = diagonalMoves * sqrt(2);

    // linear cost
    cost += std::max(xDist, yDist) - diagonalMoves;

    return cost;
}

double pathfinder::findDistanceHome(const unsigned int startingCell, const std::vector<cellData> &cellDataList, const nav_msgs::msg::OccupancyGrid &map) {
    unsigned int currentCell = startingCell;

    if (cellDataList.size() == 0) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Empty cell data");
        return -1;
    }

    if (cellDataList.at(currentCell).parentCell == startingCell) {
        return 0;
    }


    double distance = 0;
    unsigned int parentCell = cellDataList.at(currentCell).parentCell;

    while (parentCell != currentCell) {
        // check if current cell valid
        if (currentCell >= map.data.size()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Current cell is out of range. Result: " << currentCell << ". Expected: [0, " << map.data.size() << "]");
            return -1;
        }

        // find direction of cell
        int indexDelta = parentCell - currentCell;

        if (abs(indexDelta) == 1 || abs(indexDelta) == map.info.width) {
            // linear movement
            distance += 1;
        } else if (abs(indexDelta) == map.info.width + 1 || abs(indexDelta) == map.info.width - 1) {
            // diagonal
            distance += sqrt(2);
        }

        // update cell
        currentCell = parentCell;
        parentCell = cellDataList.at(currentCell).parentCell;
    }

    return distance;
}

geometry_msgs::msg::PoseArray pathfinder::AStar(const geometry_msgs::msg::Pose startPose, const geometry_msgs::msg::Pose goalPose) {
    nav_msgs::msg::OccupancyGrid map = getMap();
    geometry_msgs::msg::PoseArray path;

    // check if start and goal poses are valid
    if (!validatePose(startPose, map)) {
        return path;
    }
    
    if (!validatePose(goalPose, map)) {
        return path;
    }

    // get start and end cells
    unsigned int startCell = cart2cell(startPose, map);
    unsigned int goalCell = cart2cell(goalPose, map);

    // check if start and goal cells are the same
    if (startCell == goalCell) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start and goal poses are in the same cell");
        path.poses.push_back(goalPose);
        return path;
    }

    // generate new cell data
    std::vector<cellData> cellDataVector;

    for (unsigned int i = 0; i < map.data.size(); i++) {
        cellData tempCell;
        tempCell.index = i;
        tempCell.visited = false;
        tempCell.occupancy = static_cast<int>(map.data.at(i));
        tempCell.parentCell = UINT_MAX;
        tempCell.gCost = INFINITY;
        tempCell.hCost = INFINITY;
        tempCell.fCost = INFINITY;

        cellDataVector.push_back(tempCell);
    }

    // set start cell data
    cellDataVector.at(startCell).visited = true;
    cellDataVector.at(startCell).parentCell = startCell;
    cellDataVector.at(startCell).gCost = 0;
    cellDataVector.at(startCell).hCost = 0;
    cellDataVector.at(startCell).fCost = cellDataVector.at(startCell).gCost + cellDataVector.at(startCell).hCost;

    // comparison for priority queue
    struct Compare {
        bool operator()(const cellData &a, const cellData &b) {
            return a.fCost > b.fCost;
        }
    };

    // open list
    std::priority_queue<cellData, std::vector<cellData>, Compare> openList;
    openList.push(cellDataVector.at(startCell));

    // closed list - where we have been
    std::vector<int> closedList;

    bool goalFound = false;
    while (!openList.empty()) {
        // new cycle
        cellData currentCell = openList.top();
        openList.pop();
        closedList.push_back(currentCell.index);

        // find index's around current cell
        std::array<int, 8> adjacentCells = findAdjacentCells(currentCell.index, map);

        for (int i = 0; i < 8; i++) {
            // validity check
            if (adjacentCells[i] == -1) {
                continue;
            }

            // check if goal
            if (static_cast<unsigned int>(adjacentCells[i]) == goalCell) {
                goalFound = true;
            }

            // check if cell is in closed list
            if (std::find(closedList.begin(), closedList.end(), adjacentCells[i]) != closedList.end()) {
                continue;
            }

            // check if cell occupied
            if (cellDataVector.at(adjacentCells[i]).occupancy >= CELL_OCCUPIED_THRESHOLD) {
                closedList.push_back(adjacentCells[i]);
                continue;
            }

            // set h cost
            cellDataVector.at(adjacentCells[i]).hCost = findGridDistance(adjacentCells[i], goalCell, map);

            // calculate g and f costs from current cell
            double newGCost = findGridDistance(currentCell.index, adjacentCells[i], map);
            newGCost += currentCell.gCost;
            double newFCost = cellDataVector.at(adjacentCells[i]).hCost + newGCost;
            
            // make current cell new parent if new cost is lower
            if (newFCost < cellDataVector.at(adjacentCells[i]).fCost) {
                // update cell data
                cellDataVector.at(adjacentCells[i]).parentCell = currentCell.index;
                cellDataVector.at(adjacentCells[i]).gCost = newGCost;
                cellDataVector.at(adjacentCells[i]).fCost = newFCost;

                // if already on open list, update it
                if (cellDataVector.at(adjacentCells[i]).visited) {
                    // force priority queue to update
                    std::vector<cellData> tempList;
                    cellData tempCell = openList.top();
                    openList.pop();

                    // find edited cell
                    while (tempCell.index != cellDataVector.at(adjacentCells[i]).index) {
                        tempList.push_back(tempCell);
                        tempCell = openList.top();

                        if (openList.empty()) {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find cell in open list");
                            break;
                        }
                        openList.pop();
                    }

                    // remove edited cell
                    tempList.push_back(tempCell);
                    tempCell = openList.top();
                    if (!openList.empty()) {
                        openList.pop();
                    }

                    // return cells back to list
                    for (unsigned int j = 0; j < tempList.size(); j++) {
                        openList.push(tempList.at(j));
                    }
                }
            }

            // add to open list if not already visited
            if (!cellDataVector.at(adjacentCells[i]).visited) {
                cellDataVector.at(adjacentCells[i]).visited = true;
                openList.push(cellDataVector.at(adjacentCells[i]));
            }
        }

        // if goal found, clear open list to end loop
        if (goalFound) {
            // Clear the queue
            std::priority_queue<cellData, std::vector<cellData>, Compare>().swap(openList);
        }
    }

    // check if goal found
    if (!goalFound) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Cannot find path");
        return path;
    }

    // generate path (this is generating backwards)
    path.poses.push_back(goalPose);

    cellData currentCell = cellDataVector.at(goalCell);
    geometry_msgs::msg::Pose pose = goalPose;

    while (currentCell.parentCell != startCell) {
        // skip first cell to prevent overshooting goal if in first part of cell
        if (currentCell.index == goalCell) {
            currentCell = cellDataVector.at(currentCell.parentCell);
            pose = cell2cart(currentCell.parentCell, map);
            continue;
        }

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
    
    // simplify path
    // remove redundant points i.e. only needs the start and end point of a line, not the middle ones

    publishMarkers(path);

    return path;
}