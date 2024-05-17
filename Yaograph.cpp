#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <random>
#include <ctime>
#include <functional>
#include <cstdio>
#include <cstdlib>

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
class YaoGraph {
public:
    int pointCount = 0;
    int coneCount = 0;         // # of cones for each point (=k)
    double stretchFactor = -1;
    vector<Point> points = {}; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list = {}; // 인접 리스트

    // path
    vector<int> graphPath;
    double directDistance;
    double graphDistance;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> targetPoints = {};
    vector<int> targets = {};


public:
    // YaoGraph(int n, int k) : pointCount(n + 1), adj_list(n + 4), coneCount(k) {} // 점의 수 n으로 그래프 초기화
    YaoGraph(int n, int k) : pointCount(n), coneCount(k) {
        adj_list.assign(n, {});
        graphPath = {}; directDistance = -1; graphDistance = -1;
    }
    YaoGraph(int k) : coneCount(k) {
        pointCount = 0;
        graphPath = {}; directDistance = -1; graphDistance = -1;
    }
    void addPoint(Point p);
    void print();
};

void YaoGraph::addPoint(Point p) {
    pointCount += 1;
    points.push_back(p);
    this->adj_list.push_back({});
}

void YaoGraph::print() {
    for (int i = 0; i < this->adj_list.size(); i++) {
        auto& list = this->adj_list[i];
        for (auto& j : list) {
            cout << "edge (" << i << ", ";
            cout << j.dest << ")" << endl;
        }
        
    }
}

// 두 점 사이의 거리를 계산하는 함수
double distance(const Point& p1, const Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// 두 점의 상대 각도를 계산하는 함수 (0~360도 사이의 값)
double computeRelativeAngle(const Point& p1, const Point& p2) {
    double angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / M_PI;
    return angle >= 0 ? angle : 360.0 + angle; // 음수일 경우 양수로 변환
}

//// 두 각도의 차이를 계산하는 함수 (0 ~ 360도 사이의 값)
//double computeAngleDifference(double angle1, double angle2) {
//    double diff = angle1 - angle2;
//    if (diff > 360.0)
//        diff -= 360.0;
//    else if (diff < -180.0)
//        diff += 360.0;
//    return diff;
//}

// shortest_paths[P] = 0으로 초기화하고, 나머지는 무한대로 설정하는 함수
void initializeShortestPaths(int size, unordered_map<int, double>& shortest_paths, int P) {
    shortest_paths.clear();
    for (int i = 0; i <= size; i++)
        shortest_paths[i] = 9999999;
    shortest_paths[P] = 0;
}

// Dijkstra 알고리즘을 이용하여 최단 경로 및 stretch factor를 계산하는 함수
unordered_map<int, double> computeShortestPaths(YaoGraph& G, int P) {
    unordered_map<int, double> shortest_paths;
    initializeShortestPaths(G.pointCount, shortest_paths, P);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({ 0, P });

    vector<int> prev;
    prev.assign(G.pointCount, -1); prev[P] = P;

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
                prev[next_point] = current_point;

                double directDistance = distance(G.points[P], G.points[next_point]);
                double graphDistance = shortest_paths[next_point];

                if (G.stretchFactor < graphDistance / directDistance) {
                    G.stretchFactor = graphDistance / directDistance;

                    // graphPath, graphDistance, directDistance 업데이트
                    G.directDistance = directDistance;
                    G.graphDistance = graphDistance;
                    vector<int>().swap(G.graphPath);

                    int cur = next_point;
                    while (cur != prev[cur]) {
                        G.graphPath.insert(G.graphPath.begin(), cur);
                        cur = prev[cur];
                    }
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
            shortest_paths = computeShortestPaths(G, G.targets[i]);

        printf("Stretch Factor with reference angle %f: %f\n", current_angle, G.stretchFactor);
        current_angle += rotate;
    }
}

void computeYaoGraph(YaoGraph& G, double referenceAngle) {
    // adjacency list를 초기화
    for (auto& list : G.adj_list) vector<Edge>().swap(list);

    int cone = G.coneCount;
    int pt = G.pointCount;
    double oneRound = 360.0 / double(cone);
    
    // k개의 cone 각각에 이웃 point 저장(각 group당 하나씩 선택됨)
    // pair는 (이웃 point의 index, 거리)로 이루어짐
    vector<vector<pair<double, int>>> conePoints = {};
    conePoints.assign(cone, {});

    // 각 point마다
    for (int i = 0; i < pt; i++) {
        vector<pair<int, double>> neighbors;
        // relative angle을 계산함
        for (int j = 0; j < pt; j++) {
            if (i == j) continue;
            double angle = computeRelativeAngle(G.points[i], G.points[j]);
            neighbors.push_back(make_pair(j, angle));
        }

        // 내부 if문에서는 strict inequality가 맞음
        for (auto& pr : neighbors) { if (pr.second < referenceAngle) pr.second += 360.0; }

        // 몫과 나머지 계산하여 group 선택
        // 나머지 연산을 함으로써 자동으로 시작 - closed interval, 끝 - open interval이 결정됨
        for (auto& pr : neighbors) { 
            int group = (pr.second - referenceAngle) / oneRound;
            conePoints[group % conePoints.size()].push_back(make_pair(distance(G.points[i], G.points[pr.first]), pr.first));
        }

        // 각 group 내에서 제일 가까운 친구 계산해서 edge에 더함 (sorting 시 첫번째가 되는 것)
        for (auto& group : conePoints) {
            if (group.empty()) continue;

            sort(group.begin(), group.end());
            Edge E(group[0].second, group[0].first);
            G.adj_list[i].push_back(E);
        }
        
    }
    
}

int main() {
    // 랜덤 시드 설정
    // srand(time(NULL));
    // 
    // 
    int numPoints = 12;
    int numCones = 4;

    // point 10개, cone 3개인 YaoGraph 생성
    // YaoGraph G(10, 3);
    YaoGraph G(3);

    mt19937 engine((unsigned int)time(NULL));   
    uniform_int_distribution<int> distribution(0, 1);
    auto generator = bind(distribution, engine);

    double max = 32767;

    // 랜덤 점들 생성
    for (int i = 0; i < numPoints; ++i) {
        double x = rand() / max;
        double y = rand() / max;
        Point p(x, y);
        G.addPoint(p);
    }

    //double curAngle = 0.0;
    //double oneRound = 1.0;
    //double finish = double(360) / double(numCones);
    // 다음 angle로 이동
    // curAngle += oneRound;

    vector<double> angles;

    // pairwise angle 계산 후 sorting
    for (int i = 0; i < G.pointCount; i++) {
        for (int j = 0; j < G.pointCount; j++) {
            if (i == j) continue;
            
            double angle = computeRelativeAngle(G.points[i], G.points[j]);
            angles.push_back(angle);
        }
    }

    double start = 0.0; angles.push_back(start);
    double end = 360.0 / double(numCones); angles.push_back(end);

    sort(angles.begin(), angles.end());

    vector<double> midAngles;
    for (int i = 0; i < angles.size() - 1; i++) {
        midAngles.push_back((angles[i] + angles[i + 1]) / 2);
    }

    int totalChangeNum = 0;
    int changeNum = 0;
    double prevFactor = -1;
    // double curFactor = -1;
    double eps = 0.001;

    for (auto& midAngle : midAngles) {
        if (midAngle >= end) continue;

        totalChangeNum++;

        computeYaoGraph(G, midAngle);
        // G.print();

        // stretch factor 초기화
        G.stretchFactor = -1.0;

        // stretch factor 재계산
        for (int j = 0; j < G.pointCount; j++) {
            computeShortestPaths(G, j);
        }

        if (prevFactor == -1) prevFactor = G.stretchFactor;
        else if (abs(prevFactor - G.stretchFactor) > eps) {
            prevFactor = G.stretchFactor;
            changeNum++;
        }

        printf("Stretch Factor with reference angle %f: %f\n", midAngle, G.stretchFactor);
        if (!G.graphPath.empty()) {
            cout << "Worst pair: (" << G.graphPath[0] << ", " << G.graphPath[G.graphPath.size() - 1] << ")" << endl;
            cout << "Direct Distane: " << G.directDistance << ", Graph Distance: " << G.graphDistance << endl;
        }

        cout << endl;
    }

    cout << "Total stretch factor changes: " << changeNum << " out of " << totalChangeNum << endl;

}

//int main(void) {
//    srand(time(NULL));
//    YaoGraph G(10, 3);
//    Point origin(0, 0);
//    G.points.push_back(origin);
//
//    for (int i = 0; i < 10; i++) {
//        double new_x = rand() % 10;
//        double new_y = rand() % 10;
//        Point newPoint(new_x, new_y);
//
//        G.points.push_back(newPoint);
//    }
//
//    for (int i = 1; i <= 10; i++) {
//        vector<Edge> adjList;
//        for (int j = i + 1; j <= 10; j++) {
//            Edge edge(j, distance(G.points[i], G.points[j]));
//            adjList.push_back(edge);
//
//            //if (rand() % 2 == 0) {
//            //    Edge edge(j, distance(G.points[i], G.points[j]));
//            //    adjList.push_back(edge);
//            //}
//        }
//        G.adj_list[i] = adjList;
//    }
//
//    rotateYaoGraph(G, 0, M_PI / 12);
//    return 0;
//}