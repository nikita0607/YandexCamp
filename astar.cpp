#include <map>
#include <algorithm>
#include<cmath>
#include<limits>
#include<queue>
#include<vector>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace std::chrono;

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;


class AStarPath {
public:
    AStarPath(double robot_radius, double grid_size, const std::vector<int>& x_obstacle, const std::vector<int>& y_obstacle, bool show_animation = true)
        : robot_radius(robot_radius), grid_size(grid_size), show_animation(show_animation) {

        createObstacleMap(x_obstacle, y_obstacle);
        path = getPath();
    }

    struct Node {
        int x, y;
        double cost;
        int path;
        Node() : x(0), y(0), cost(0.0), path(-1) {}
        Node(int x, int y, double cost, int path) : x(x), y(y), cost(cost), path(path) {}
    };

    double calcHeuristic(const Node& n1, const Node& n2) {
        double weight = 1.0;
        return weight * std::sqrt((n1.x - n2.x) * (n1.x - n2.x) + (n1.y - n2.y) * (n1.y - n2.y));
    }

    double calcGridPosition(int index, int min_position) {
        return index * grid_size + min_position;
    }

    int calcXY(double position, double min_position) {
        return static_cast<int>(std::round((position - min_position) / grid_size));
    }

    int calcGridIndex(const Node& node) {
        return (node.y - y_min) * x_width + (node.x - x_min);
    }

    bool checkValidity(const Node& node) {
        double x_position = calcGridPosition(node.x, x_min);
        double y_position = calcGridPosition(node.y, y_min);

        if (x_position < x_min || y_position < y_min || x_position >= x_max || y_position >= y_max) {
            return false;
        }
        if (obstacle_pos[node.x][node.y]) {
            return false;
        }
        return true;
    }

    void createObstacleMap(const std::vector<int>& x_obstacle, const std::vector<int>& y_obstacle) {
        x_min = *std::min_element(x_obstacle.begin(), x_obstacle.end());
        y_min = *std::min_element(y_obstacle.begin(), y_obstacle.end());
        x_max = *std::max_element(x_obstacle.begin(), x_obstacle.end());
        y_max = *std::max_element(y_obstacle.begin(), y_obstacle.end());

        x_width = static_cast<int>((x_max - x_min) / grid_size);
        y_width = static_cast<int>((y_max - y_min) / grid_size);
        obstacle_pos = std::vector<std::vector<bool>>(x_width, std::vector<bool>(y_width, false));

        for (int i = 0; i < x_width; ++i) {
            double x = calcGridPosition(i, x_min);
            for (int j = 0; j < y_width; ++j) {
                double y = calcGridPosition(j, y_min);
                for (size_t k = 0; k < x_obstacle.size(); ++k) {
                    double d = std::sqrt((x_obstacle[k] - x) * (x_obstacle[k] - x) + (y_obstacle[k] - y) * (y_obstacle[k] - y));
                    if (d <= robot_radius) {
                        obstacle_pos[i][j] = true;
                        break;
                    }
                }
            }
        }
    }

    std::vector<std::vector<double>> getPath() {
        return {
            {1, 0, 1},
            {0, 1, 1},
            {-1, 0, 1},
            {0, -1, 1},
            {-1, -1, std::sqrt(2)},
            {-1, 1, std::sqrt(2)},
            {1, -1, std::sqrt(2)},
            {1, 1, std::sqrt(2)}
        };
    }

    std::pair<std::vector<int>, std::vector<int>> calcFinalPath(Node& end_node, const std::map<int, Node>& record_closed) {
        std::vector<int> x_out_path = { static_cast<int>(calcGridPosition(end_node.x, x_min)) };
        std::vector<int> y_out_path = { static_cast<int>(calcGridPosition(end_node.y, y_min)) };
        int path = end_node.path;

        while (path != -1) {
            const Node& n = record_closed.at(path);
            x_out_path.push_back(static_cast<int>(calcGridPosition(n.x, x_min)));
            y_out_path.push_back(static_cast<int>(calcGridPosition(n.y, y_min)));
            path = n.path;
        }

        return { x_out_path, y_out_path };
    }

    std::pair<std::vector<int>, std::vector<int>> aStarSearch(int start_x, int start_y, int end_x, int end_y) {
        Node start_node(calcXY(start_x, x_min), calcXY(start_y, y_min), 0.0, -1);
        Node end_node(calcXY(end_x, x_min), calcXY(end_y, y_min), 0.0, -1);

        std::map<int, Node> record_open, record_closed;
        record_open[calcGridIndex(start_node)] = start_node;

        while (true) {
            if (record_open.empty()) {
                std::cout << "Check Record Validity" << std::endl;
                break;
            }

            auto iter = std::min_element(record_open.begin(), record_open.end(), [&](const auto& lhs, const auto& rhs) {
                return lhs.second.cost + this->calcHeuristic(end_node, lhs.second) < rhs.second.cost + this->calcHeuristic(end_node, rhs.second);
            });

            Node cost_collection = iter->second;

            if (cost_collection.x == end_node.x && cost_collection.y == end_node.y) {
                std::cout << "Finished!" << std::endl;
                end_node.path = cost_collection.path;
                end_node.cost = cost_collection.cost;
                break;
            }

            record_open.erase(iter);
            record_closed[calcGridIndex(cost_collection)] = cost_collection;

            for (const auto& p : path) {
                Node node(cost_collection.x + p[0], cost_collection.y + p[1], cost_collection.cost + p[2], calcGridIndex(cost_collection));
                int idx_node = calcGridIndex(node);

                if (!checkValidity(node)) continue;
                if (record_closed.count(idx_node)) continue;

                if (!record_open.count(idx_node) || record_open[idx_node].cost > node.cost) {
                    record_open[idx_node] = node;
                }
            }
        }

        return calcFinalPath(end_node, record_closed);
    }

    private:
        double robot_radius, grid_size;
        int x_min, y_min, x_max, y_max, x_width, y_width;
        bool show_animation;
        std::vector<std::vector<bool>> obstacle_pos;
        std::vector<std::vector<double>> path;
};


void readFileToVector(const string& filename, vector<int>& vec) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    int number;
    while (file >> number) {
        vec.push_back(number);
    }
}

void readFileToVector(const string& filename, vector<float>& vec) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    float number;
    while (file >> number) {
        vec.push_back(number);
    }
}


void writeVectorToFile(const std::vector<int>& vec, const std::string& filename, bool append = false) {
    std::ofstream outFile;
    if (append) {
        outFile.open(filename, std::ios::app); // Open in append mode
    } else {
        outFile.open(filename);
    }

    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
        outFile << vec[i];
        if (i < vec.size() - 1) {
            outFile << " ";
        }
    }

    outFile << std::endl; // Add a newline at the end of the vector output
    outFile.close(); // Close the file
}


int main(int argc,char* argv[]) {

    vector<int> x_obstacle;
    vector<int> y_obstacle;
    vector<float> meta;

    auto start = high_resolution_clock::now();

    readFileToVector("x_obstacles.txt", x_obstacle);
    readFileToVector("y_obstacles.txt", y_obstacle);
    readFileToVector("meta.txt", meta);

    float robot_radius = meta[0];
    float grid_size = meta[1];

    int start_x = (int)(meta[2]);
    int start_y = (int)(meta[3]);
    int end_x =   (int)(meta[4]);
    int end_y =   (int)(meta[5]);

    // float robot_radius = 7.8;
    // float grid_size = 4.0;

    // int start_x = 225;
    // int start_y = 140;
    // int end_x =   125;
    // int end_y =   50;

    AStarPath astar(robot_radius, grid_size, x_obstacle, y_obstacle, false);
    auto result = astar.aStarSearch(start_x, start_y, end_x, end_y);

    writeVectorToFile(result.first, "x_out_path.txt");
    writeVectorToFile(result.second, "y_out_path.txt");

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    ms d = duration_cast<ms>(duration);
    cout << "path built in: " << d.count() << "ms" << endl;

    cout << x_obstacle.size() << endl;

    return 0;
}