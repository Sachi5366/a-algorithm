#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

const int ROW = 5;
const int COL = 5;

// Grid directions (up, right, down, left)
int dx[] = {-1, 0, 1, 0};
int dy[] = {0, 1, 0, -1};

// Manhattan Distance heuristic
int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// Check if the cell is within the grid and not an obstacle
bool isValid(int x, int y, vector<vector<int>> &grid) {
    return x >= 0 && x < ROW && y >= 0 && y < COL && grid[x][y] == 0;
}

// Structure for A* node
struct Node {
    int x, y;
    int g, h;
    vector<pair<int, int>> path;

    Node(int x, int y, int g, int h, vector<pair<int, int>> path)
        : x(x), y(y), g(g), h(h), path(path) {}

    int f() const {
        return g + h;
    }

    // Custom comparator for min-heap
    bool operator>(const Node &other) const {
        return f() > other.f();
    }
};

// A* Pathfinding Function
void aStar(vector<vector<int>> &grid, pair<int, int> start, pair<int, int> goal) {
    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    vector<vector<bool>> visited(ROW, vector<bool>(COL, false));

    vector<pair<int, int>> initialPath = {start};
    int startH = heuristic(start.first, start.second, goal.first, goal.second);
    openSet.push(Node(start.first, start.second, 0, startH, initialPath));

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        int x = current.x;
        int y = current.y;

        if (visited[x][y])
            continue;
        visited[x][y] = true;

        // Goal reached
        if (x == goal.first && y == goal.second) {
            cout << "Path found:\n";
            for (auto &cell : current.path) {
                cout << "(" << cell.first << "," << cell.second << ") ";
            }
            cout << endl;
            return;
        }

        // Explore neighbors
        for (int dir = 0; dir < 4; dir++) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];

            if (isValid(nx, ny, grid) && !visited[nx][ny]) {
                vector<pair<int, int>> newPath = current.path;
                newPath.push_back({nx, ny});
                int gNew = current.g + 1;
                int hNew = heuristic(nx, ny, goal.first, goal.second);
                openSet.push(Node(nx, ny, gNew, hNew, newPath));
            }
        }
    }

    cout << "No path found.\n";
}

// Main function
int main() {
    vector<vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    pair<int, int> start = {0, 0};
    pair<int, int> goal = {4, 4};

    aStar(grid, start, goal);

    return 0;
}
