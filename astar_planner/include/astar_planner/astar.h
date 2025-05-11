#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>

struct Node {
    int x, y;
    float g, f;
    Node(int _x, int _y) : x(_x), y(_y), g(1e9), f(1e9) {}
    bool operator<(const Node& other) const { return f > other.f; } // For min-heap
};

class AStar {
public:
    AStar(const nav_msgs::OccupancyGrid& map);
    std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal);

private:
    int width, height;
    std::vector<int8_t> grid;

    float heuristic(int x1, int y1, int x2, int y2);
    bool isValid(int x, int y, int parent_x = -1, int parent_y = -1); // 防止斜穿障碍
};

#endif