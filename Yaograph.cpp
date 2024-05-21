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
    pair<int, int> factorPoint;
    vector<Point> points = {}; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list = {}; // 인접 리스트

    YaoGraph(int n, int k) : pointCount(n), adj_list(n), coneCount(k) {} // 점의 수 n으로 그래프 초기화
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
void initializeShortestPaths(YaoGraph& G, unordered_map<int, double>& shortest_paths, int P) {
    shortest_paths.clear();
    for (int i = 0; i < G.pointCount; i++)
        shortest_paths[i] = 9999999;
    shortest_paths[P] = 0;
}

// Dijkstra 알고리즘을 이용하여 최단 경로 및 stretch factor를 계산하는 함수
unordered_map<int, double> computeShortestPaths(YaoGraph& G, int P, double angle) {
    unordered_map<int, double> shortest_paths;
    initializeShortestPaths(G, shortest_paths, P);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({ 0, P });

    while (!pq.empty()) {
        double dist = pq.top().first;
        int current_point = pq.top().second;
        pq.pop();

        for (const auto& edge : G.adj_list[current_point]) {
            int next_point = edge.dest;
            double weight = edge.weight;

            if (shortest_paths[current_point] + weight < shortest_paths[next_point]) {
                shortest_paths[next_point] = shortest_paths[current_point] + weight;
                pq.push({ shortest_paths[next_point], next_point });

                if (G.stretchFactor < shortest_paths[next_point] / distance(G.points[P], G.points[next_point])) {
                    G.stretchFactor = shortest_paths[next_point] / distance(G.points[P], G.points[next_point]);
                    G.factorPoint = make_pair(P, next_point);
                }
            }
        }
    }

    return shortest_paths;
}

int insert(YaoGraph& G, int centerPoint, double rightRayAngle) {
    double minDist = 9999999;
    double minPoint = -1;
    for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
        if (centerPoint == curPoint)
            continue;

        double relative_angle = computeRelativeAngle(G.points[centerPoint], G.points[curPoint]);
        if (relative_angle < rightRayAngle)
            relative_angle += rightRayAngle;

        double angle_diff = relative_angle - rightRayAngle;

        if (angle_diff >= 0 && angle_diff < 360 / G.coneCount) {
            double dist = distance(G.points[centerPoint], G.points[curPoint]);
            if (minDist > dist) {
                minDist = dist;
                minPoint = curPoint;
            }
        }
    }
    return minPoint;
}

// Yao Graph의 ray를 회전하며 stretch factor를 매번 계산하는 함수
void rotateYaoGraph(YaoGraph& G, double rightRayAngle, double rotate) {
    double current_angle = rightRayAngle;
    unordered_map<int, double> shortest_paths;

    while (current_angle < rightRayAngle + 360 / G.coneCount) {
        // 해당 영역 안의 점들을 PQ에 삽입
        // memset(G.adjList, 0, sizeof(G.adjList));
        for (int point = 0; point < G.pointCount; point++)
            G.adj_list[point].clear();

        for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
            // vector<Edge> adjList;
            // 각각의 cone마다 가장 가까운 점을 edge로 잇는 작업
            for (int rightAngle = current_angle; rightAngle < 360 + current_angle; rightAngle += 360 / G.coneCount) {
                int destPoint = insert(G, curPoint, rightAngle);
                if (destPoint == -1)
                    continue;

                double dist = distance(G.points[curPoint], G.points[destPoint]);
                Edge cur_newEdge(destPoint, dist);
                Edge dest_newEdge(curPoint, dist);

                G.adj_list[curPoint].push_back(cur_newEdge);
                G.adj_list[destPoint].push_back(dest_newEdge);
            }

            // G.adj_list[curPoint] = adjList;
        }

        G.stretchFactor = -1;
        for (int cur = 0; cur < G.pointCount; cur++) {
            shortest_paths.clear();
            shortest_paths = computeShortestPaths(G, cur, current_angle);
        }

        printf("Stretch Factor with reference angle %f: %f\n", current_angle, G.stretchFactor);
        printf("Points: (%.f, %.f) and (%.f, %.f)\n\n\n", G.points[G.factorPoint.first].x, G.points[G.factorPoint.first].y,
                                            G.points[G.factorPoint.second].x, G.points[G.factorPoint.second].y);
        current_angle += rotate;
    }
}


int main(void) {
    srand(time(NULL));
    YaoGraph G(100, 8);

    for (int i = 0; i < G.pointCount; i++) {
        double new_x = rand() % 100 - 50;
        double new_y = rand() % 100 - 50;
        Point newPoint(new_x, new_y);

        G.points.push_back(newPoint);
    }

    rotateYaoGraph(G, 0, M_PI / 12);
    return 0;
}