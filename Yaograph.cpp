#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <random>

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
    int pointCount = 0;
    int coneCount = 0;         // # of cones for each point (=k)
    double stretchFactor = -1;
    vector<Point> points = {}; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list = {}; // 인접 리스트
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> targetPoints = {};
    vector<int> targets = {};

    YaoGraph(int n, int k) : pointCount(n + 1), adj_list(n + 4), coneCount(k) {} // 점의 수 n으로 그래프 초기화
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

// 두 각도의 차이를 계산하는 함수 (0 ~ 360도 사이의 값)
double computeAngleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    if (diff > 360.0)
        diff -= 360.0;
    else if (diff < -180.0)
        diff += 360.0;
    return diff;
}

// shortest_paths[P] = 0으로 초기화하고, 나머지는 무한대로 설정하는 함수
void initializeShortestPaths(unordered_map<int, double>& shortest_paths, int P) {
    shortest_paths.clear();
    for (int i = 0; i <= 10; i++)
        shortest_paths[i] = 9999999;
    shortest_paths[P] = 0;
}

// Dijkstra 알고리즘을 이용하여 최단 경로 및 stretch factor를 계산하는 함수
unordered_map<int, double> computeShortestPaths(YaoGraph& G, int P, double angle) {
    unordered_map<int, double> shortest_paths;
    initializeShortestPaths(shortest_paths, P);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({ 0, P });

    while (!pq.empty()) {
        double dist = pq.top().first;
        int current_point = pq.top().second;
        pq.pop();

        for (const auto& edge : G.adj_list[current_point]) {
            int next_point = edge.dest;
            double weight = edge.weight;

            // cone ray 내부에 있는 점들만 고려
            if (find(G.targets.begin(), G.targets.end(), next_point) != G.targets.end()) {
                if (shortest_paths[current_point] + weight < shortest_paths[next_point]) {
                    shortest_paths[next_point] = shortest_paths[current_point] + weight;
                    pq.push({ shortest_paths[next_point], next_point });

                    if (G.stretchFactor < shortest_paths[next_point] / distance(G.points[P], G.points[next_point]))
                        G.stretchFactor = shortest_paths[next_point] / distance(G.points[P], G.points[next_point]);
                }

            }
        }
    }

    return shortest_paths;
}

// Yao Graph의 ray를 회전하며 stretch factor를 매번 계산하는 함수
void rotateYaoGraph(YaoGraph& G, double rightRayAngle, double rotate) {
    double current_angle = rightRayAngle;
    unordered_map<int, double> shortest_paths;

    while (current_angle < rightRayAngle + 360 / G.coneCount) {
        // 해당 영역 안의 점들을 PQ에 삽입
        G.targetPoints = {};
        G.targets = {};
        G.stretchFactor = -1;
        for (int cur = 1; cur < G.pointCount; cur++) {
            double relative_angle = computeRelativeAngle(G.points[0], G.points[cur]);
            double angle_diff = relative_angle - current_angle;

            if (angle_diff >= 0 && angle_diff < 360 / G.coneCount) {
                G.targetPoints.push(make_pair(angle_diff, cur));
                G.targets.push_back(cur);
            }
        }

        for (int i = 0; i < G.targets.size(); i++)
            shortest_paths = computeShortestPaths(G, G.targets[i], current_angle);

        printf("Stretch Factor with reference angle %f: %f\n", current_angle, G.stretchFactor);
        current_angle += rotate;
    }
}


int main(void) {
    srand(time(NULL));
    YaoGraph G(10, 3);
    Point origin(0, 0);
    G.points.push_back(origin);

    for (int i = 0; i < 10; i++) {
        double new_x = rand() % 10;
        double new_y = rand() % 10;
        Point newPoint(new_x, new_y);

        G.points.push_back(newPoint);
    }

    for (int i = 1; i <= 10; i++) {
        vector<Edge> adjList;
        for (int j = i + 1; j <= 10; j++) {
            if (rand() % 2 == 0) {
                Edge edge(j, distance(G.points[i], G.points[j]));
                adjList.push_back(edge);
            }
        }
        G.adj_list[i] = adjList;
    }

    rotateYaoGraph(G, 0, M_PI / 12);
    return 0;
}