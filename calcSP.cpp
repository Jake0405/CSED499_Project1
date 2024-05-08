#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <limits>

using namespace std;

// 그래프의 각 점을 나타내는 구조체
struct Point {
    int isValid = 1;
    double x, y; // x와 y 좌표

    Point(double _x, double _y) : x(_x), y(_y) {}
};

// 그래프의 간선을 나타내는 구조체
struct Edge {
    int dest; // 목적지 점의 인덱스
    double weight; // 간선의 가중치

    Edge(int _dest, double _weight) : dest(_dest), weight(_weight) {}
};

// Yao 그래프를 나타내는 구조체
struct YaoGraph {
    int coneCount;         // # of cones for each point (=k)
    int stretchFactor = INT_MIN;
    vector<Point> points; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list; // 인접 리스트
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> targetPoints;

    YaoGraph(int n) : adj_list(n) {} // 점의 수 n으로 그래프 초기화
};

// 두 점 사이의 거리를 계산하는 함수
double distance(const Point& p1, const Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// 두 점의 상대 각도를 계산하는 함수 (0~360도 사이의 값)
double computeRelativeAngle(const Point& p1, const Point& p2) {
    double angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / M_PI;
    return angle >= 0 ? angle : 360.0 + angle; // 음수일 경우 양수로 변환
}

// 두 각도의 차이를 계산하는 함수 (-180 ~ 180도 사이의 값)
double computeAngleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    if (diff > 180.0)
        diff -= 360.0;
    else if (diff < -180.0)
        diff += 360.0;
    return diff;
}

// shortest_paths[P] = 0으로 초기화하고, 나머지는 무한대로 설정하는 함수
void initializeShortestPaths(unordered_map<int, double>& shortest_paths, int P) {
    shortest_paths.clear();
    shortest_paths[P] = 0;
}

// Dijkstra 알고리즘을 이용하여 최단 경로를 계산하는 함수
unordered_map<int, double> computeShortestPaths(YaoGraph& G, int P, double angle) {
    unordered_map<int, double> shortest_paths;
    initializeShortestPaths(shortest_paths, P);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({0, P});

    while (!pq.empty()) {
        auto [dist, current_point] = pq.top();
        pq.pop();

        for (const auto& edge : G.adj_list[current_point]) {
            int next_point = edge.dest;
            double weight = edge.weight;

            if (G.points[next_point].isValid == 0)
                continue;

            double relative_angle = computeRelativeAngle(G.points[P], G.points[next_point]);
            double angle_diff = computeAngleDifference(angle, relative_angle);

            // cone ray 내부에 있는 점들만 고려
            if (angle_diff >= -180 / G.coneCount && angle_diff < 180 / G.coneCount) {
                if (shortest_paths[current_point] + weight < shortest_paths[next_point]) {
                    G.targetPoints.push({angle_diff, next_point});
                    shortest_paths[next_point] = shortest_paths[current_point] + weight;
                    pq.push({shortest_paths[next_point], next_point});

                    if (G.stretchFactor > shortest_paths[next_point] / distance(P, next_point))
                        G.stretchFactor = shortest_paths[next_point] / distance(P, next_point);
                }
            }
        }
    }

    return shortest_paths;
}

void rotateYaoGraph(YaoGraph& G, int P, double rightRayAngle, double rotate) {
    double current_angle = rightRayAngle;
    unordered_map<int, double> shortest_paths;

    while (current_angle < rightRayAngle + 360 / G.coneCount) {
        while (!G.targetPoints.empty()) {
            auto [angle, target_point] = G.targetPoints.top();
            double angle_diff = computeAngleDifference(angle + rightRayAngle, current_angle);

            if (angle_diff >= rotate / 2)
                break;

            G.targetPoints.pop();
            G.points[target_point].isValid = 0;

            shortest_paths = computeShortestPaths(G, P, rightRayAngle);
        }

        current_angle += rotate;
    }
}

void insertion(YaoGraph& G, double x, double y, vector<Edge> adjList) {
    int curPoint = G.points.size();
    G.points.push_back(Point(x, y));
    for (auto it : adjList)
        G.adj_list[curPoint].insert(G.adj_list[curPoint].end(), adjList.begin(), adjList.end());
}
