#include "pathfinder.h"


pathfinder::pathfinder(const nav_msgs::msg::OccupancyGrid &map) : Node("pathfinder"), CELL_OCCUPIED_THRESHOLD(50), CELL_VALUE_MIN(-1), CELL_VALUE_MAX(100), map_(map) {
    
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
    int occupancy = map.data[index];

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
    // convert point to map coords
    int mapX = pose.position.x / map.info.resolution;
    int mapY = pose.position.y /map.info.resolution;

    // convert to OcupancyGrid data format
    int index = mapY * map.info.width + mapX;

    return index;
}

geometry_msgs::msg::Pose pathfinder::cell2cart(const int index, const nav_msgs::msg::OccupancyGrid &map) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = (index % map.info.width) * map.info.resolution + map.info.resolution / 2;
    pose.position.y = (index / map.info.width) * map.info.resolution + map.info.resolution / 2;

    return pose;
}

std::array<int, 8> pathfinder::findAdjacentCells(const int index, const nav_msgs::msg::OccupancyGrid &map) {
    std::array<int, 8> adjacentCells;

    // above
    adjacentCells[0] = index - map.info.width;

    // NE
    adjacentCells[1] = index - map.info.width + 1;

    // right
    adjacentCells[2] = index + 1;

    // SE
    adjacentCells[3] = index + map.info.width + 1;

    // below
    adjacentCells[4] = index + map.info.width;

    // SW
    adjacentCells[5] = index + map.info.width - 1;

    // left
    adjacentCells[6] = index - 1;

    // NW
    adjacentCells[7] = index - map.info.width - 1;

    // set all out of bounds cells to -1
    for (int i = 0; i < 8; i++) {
        if (adjacentCells[i] < 0 || adjacentCells[i] >= map.info.width * map.info.height) {
            adjacentCells[i] = -1;
        }
    }

    return adjacentCells;
}

double pathfinder::findGridDistance(const unsigned int startCell, const unsigned int goalCell, const nav_msgs::msg::OccupancyGrid &map) {
    // find manhattan distances
    unsigned int xDist = fabs(goalCell % map.info.width - startCell % map.info.width);
    unsigned int yDist = fabs(goalCell / map.info.width - startCell / map.info.width);

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
        RCLCPP_INFO_STREAM(this->get_logger(), "We are at the start already");
        return 0;
    }


    double distance = 0;
    unsigned int parentCell = cellDataList.at(currentCell).parentCell;

    while (parentCell != currentCell) {
        // check if current cell valid
        if (currentCell < 0 || currentCell >= map.data.size()) {
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

geometry_msgs::msg::PoseArray pathfinder::AStar(const geometry_msgs::msg::Pose startPose, const geometry_msgs::msg::Pose goalPose, const nav_msgs::msg::OccupancyGrid &map) {
    geometry_msgs::msg::PoseArray path;

    // check if start and goal poses are valid
    if (!validatePose(startPose, map)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Invalid start pose");
        return path;
    }
    
    if (!validatePose(goalPose, map)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Invalid goal pose");
        return path;
    }

    // get start and end cells
    int startCell = cart2cell(startPose, map);
    int goalCell = cart2cell(goalPose, map);

    // check if start and goal cells are the same
    if (startCell == goalCell) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start and goal poses are in the same cell");
        path.poses.push_back(goalPose);
        return path;
    }

    // generate new cell data
    std::vector<cellData> cellDataVector;

    for (int i = 0; i < map.data.size(); i++) {
        cellData tempCell {i, false, map.data.at(i), UINT_MAX, INFINITY, INFINITY, INFINITY};
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
        std::array<int, 8> adjacentCells = findAdjacentCells(startCell, map);

        for (int i = 0; i < 8; i++) {
            // validity check
            if (adjacentCells[i] == -1) {
                continue;
            }

            // check if goal
            if (adjacentCells[i] == goalCell) {
                // found goal - DO SOMETHING!!!
                goalFound = true;
                openList.empty();
                break;
            }

            // check if cell is in closed list
            if (std::find(closedList.begin(), closedList.end(), adjacentCells[i]) != closedList.end()) {
                continue;
            }

            // check if cell occupied
            if (cellDataVector.at(adjacentCells[i]).occupancy >= CELL_OCCUPIED_THRESHOLD) {
                continue;
            }

            // set h cost
            cellDataVector.at(adjacentCells[i]).hCost = findGridDistance(adjacentCells[i], goalCell, map);

            // calculate g and f costs from current cell
            double newGCost = findGridDistance(cellDataVector.at(adjacentCells[i]).index, currentCell.index, map);
            newGCost += findDistanceHome(currentCell.index, cellDataVector, map);
            double newFCost = cellDataVector.at(adjacentCells[i]).hCost + newGCost;

            // make current cell new parent if new cost is lower
            if (newFCost < cellDataVector.at(adjacentCells[i]).fCost) {
                cellDataVector.at(adjacentCells[i]).parentCell = currentCell.index;
                cellDataVector.at(adjacentCells[i]).gCost = newGCost;
                cellDataVector.at(adjacentCells[i]).fCost = newFCost;
            }

            // add to open list if not already visited
            if (!cellDataVector.at(adjacentCells[i]).visited) {
                cellDataVector.at(adjacentCells[i]).visited = true;
                openList.push(cellDataVector.at(adjacentCells[i]));
            }
        }
    }

    // check if goal found
    if (!goalFound) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Cannot find path");
        return path;
    }
    
    // simplify path
    // remove redundant points i.e. only needs the start and end point of a line, not the middle ones

    return path;
}