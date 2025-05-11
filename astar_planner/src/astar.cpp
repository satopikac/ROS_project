#include "astar_planner/astar.h"
#include <cmath>

AStar::AStar(const nav_msgs::OccupancyGrid& map)
    : width(map.info.width), height(map.info.height), grid(map.data.begin(), map.data.end()) {}

float AStar::heuristic(int x1, int y1, int x2, int y2) {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return dx + dy + (M_SQRT2 - 2) * std::min(dx, dy); // Octile distance
}

bool AStar::isValid(int x, int y, int parent_x, int parent_y) {
    if (x < 0 || y < 0 || x >= width || y >= height)
        return false;

    if (grid[y * width + x] != 0)
        return false;

    if (parent_x != -1 && parent_y != -1 && parent_x != x && parent_y != y) {
        int corner1 = grid[parent_y * width + x];
        int corner2 = grid[y * width + parent_x];
        if (corner1 != 0 || corner2 != 0)
            return false;
    }

    return true;
}

std::vector<std::pair<int, int>> AStar::plan(const std::pair<int, int>& start,
                                             const std::pair<int, int>& goal) {
    std::priority_queue<Node> openList;
    std::unordered_map<int, float> gScore;
    std::unordered_map<int, float> fScore;
    std::unordered_map<int, std::pair<int, int>> cameFrom;

    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] ={-1,  0,  1,-1, 1,-1, 0, 1};

    Node startNode(start.first, start.second);
    startNode.g = 0;
    startNode.f = heuristic(start.first, start.second, goal.first, goal.second);

    openList.push(startNode);
    gScore[(start.second * width) + start.first] = 0.0f;
    fScore[(start.second * width) + start.first] = startNode.f;

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.x == goal.first && current.y == goal.second) {
            std::vector<std::pair<int, int>> path;
            int key = current.y * width + current.x;
            while (cameFrom.find(key) != cameFrom.end()) {
                path.push_back({current.x, current.y});
                current.x = cameFrom[key].first;
                current.y = cameFrom[key].second;
                key = current.y * width + current.x;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isValid(nx, ny, current.x, current.y)) continue;

            float moveCost = (dx[i] == 0 || dy[i] == 0) ? 1.0 : M_SQRT2;

            int key = ny * width + nx;
            int curKey = current.y * width + current.x;

            float tentative_g = gScore[curKey] + moveCost;

            if (gScore.find(key) == gScore.end() || tentative_g < gScore[key]) {
                cameFrom[key] = {current.x, current.y};
                gScore[key] = tentative_g;
                fScore[key] = tentative_g + heuristic(nx, ny, goal.first, goal.second);
                openList.push(Node(nx, ny));
            }
        }
    }

    return {}; // No path found
}