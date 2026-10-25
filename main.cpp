#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <iomanip>

// Structure representing a coordinate point / intersection
struct Node {
    int id;
    std::string name;
    double x, y; // 2D Spatial Coordinates
};

// Structure representing a directed road segment with a weight (distance/time)
struct Edge {
    int target;
    double weight;
};

// Result structure for pathfinding
struct PathResult {
    std::vector<int> path;
    double total_cost;
    bool found;
};

class AeroRouteEngine {
private:
    std::unordered_map<int, Node> nodes_;
    std::unordered_map<int, std::vector<Edge>> adj_list_;

    // Euclidean distance heuristic for A*
    double heuristic(int u, int target) const {
        const auto& a = nodes_.at(u);
        const auto& b = nodes_.at(target);
        return std::hypot(a.x - b.x, a.y - b.y);
    }

public:
    void addNode(int id, const std::string& name, double x, double y) {
        nodes_[id] = {id, name, x, y};
    }

    void addEdge(int u, int v, double weight) {
        adj_list_[u].push_back({v, weight});
        // Assuming bidirectional roads for city navigation
        adj_list_[v].push_back({u, weight});
    }

    PathResult findShortestPath(int start, int goal) {
        if (nodes_.find(start) == nodes_.end() || nodes_.find(goal) == nodes_.end()) {
            return {{}, -1.0, false};
        }

        using PQElement = std::pair<double, int>; // {f_score, node_id}
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;

        std::unordered_map<int, double> g_score;
        std::unordered_map<int, int> came_from;

        for (const auto& pair : nodes_) {
            g_score[pair.first] = std::numeric_limits<double>::infinity();
        }

        g_score[start] = 0.0;
        open_set.push({heuristic(start, goal), start});

        while (!open_set.empty()) {
            int current = open_set.top().second;
            open_set.pop();

            if (current == goal) {
                std::vector<int> path;
                double total_cost = g_score[goal];
                int temp = goal;
                while (temp != start) {
                    path.push_back(temp);
                    temp = came_from[temp];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return {path, total_cost, true};
            }

            for (const auto& edge : adj_list_[current]) {
                double tentative_g = g_score[current] + edge.weight;
                if (tentative_g < g_score[edge.target]) {
                    came_from[edge.target] = current;
                    g_score[edge.target] = tentative_g;
                    double f_score = tentative_g + heuristic(edge.target, goal);
                    open_set.push({f_score, edge.target});
                }
            }
        }

        return {{}, -1.0, false};
    }

    void printNodeInfo(int id) const {
        if (nodes_.count(id)) {
            std::cout << "[" << id << "] " << nodes_.at(id).name 
                      << " (X: " << nodes_.at(id).x << ", Y: " << nodes_.at(id).y << ")\n";
        }
    }
};

int main() {
    std::cout << "===========================================\n";
    std::cout << "       AeroRoute: Spatial Navigation       \n";
    std::cout << "===========================================\n\n";

    AeroRouteEngine router;

    // 1. Populate map with intersections (nodes)
    router.addNode(1, "Central Station", 0.0, 0.0);
    router.addNode(2, "Tech Park", 2.0, 3.0);
    router.addNode(3, "University Campus", 5.0, 1.0);
    router.addNode(4, "Downtown Square", 4.0, 6.0);
    router.addNode(5, "International Airport", 8.0, 8.0);

    // 2. Connect intersections with weighted road segments (edges)
    router.addEdge(1, 2, 3.5); // Central Station <-> Tech Park
    router.addEdge(1, 3, 5.2); // Central Station <-> University Campus
    router.addEdge(2, 4, 3.1); // Tech Park <-> Downtown Square
    router.addEdge(3, 5, 6.0); // University Campus <-> International Airport
    router.addEdge(4, 5, 2.8); // Downtown Square <-> International Airport
    router.addEdge(2, 3, 2.5); // Tech Park <-> University Campus

    // 3. Run pathfinding query: Central Station (1) -> International Airport (5)
    int start_id = 1;
    int goal_id = 5;

    std::cout << "Calculating optimal route from Central Station [1] to International Airport [5]...\n\n";

    PathResult result = router.findShortestPath(start_id, goal_id);

    if (result.found) {
        std::cout << "✔ Route Found Successfully!\n";
        std::cout << "Total Distance Cost: " << std::fixed << std::setprecision(2) << result.total_cost << " km\n\n";
        std::cout << "Traversal Path Sequence:\n";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << "  " << (i + 1) << ". Node ID: " << result.path[i];
            if (i == 0) std::cout << " (Start)";
            else if (i == result.path.size() - 1) std::cout << " (Destination)";
            std::cout << "\n";
        }
    } else {
        std::cout << "❌ No path found between the specified locations.\n";
    }

    std::cout << "\n===========================================\n";
    return 0;
}