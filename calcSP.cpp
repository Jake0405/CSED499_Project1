#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <limits>

using namespace std;

// 그래프의 각 점을 나타내는 구조체
struct Point {
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
    vector<Point> points; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list; // 인접 리스트

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

            double relative_angle = computeRelativeAngle(G.points[current_point], G.points[next_point]);
            double angle_diff = computeAngleDifference(angle, relative_angle);

            // cone ray 내부에 있는 점들만 고려
            if (angle_diff >= -45.0 && angle_diff <= 45.0) {
                if (shortest_paths[current_point] + weight < shortest_paths[next_point]) {
                    shortest_paths[next_point] = shortest_paths[current_point] + weight;
                    pq.push({shortest_paths[next_point], next_point});
                }
            }
        }
    }

    return shortest_paths;
}
