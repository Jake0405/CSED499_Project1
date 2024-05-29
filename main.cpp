#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <functional>
#include <random>

using namespace std;

// �׷����� �� ���� ��Ÿ���� ����ü
struct Point {
    int isValid = 1;
    double x, y; // x�� y ��ǥ

    Point(double _x, double _y) : x(_x), y(_y) {}
};

// �׷����� ������ ��Ÿ���� ����ü
struct Edge {
    int dest; // ������ ���� �ε���
    double weight; // ������ ����ġ

    Edge(int _dest, double _weight) : dest(_dest), weight(_weight) {}
};

// Yao �׷����� ��Ÿ���� ����ü
struct YaoGraph {
    int pointCount = 0;
    int coneCount = 0;         // # of cones for each point (=k)
    double stretchFactor = -1;
    pair<int, int> factorPoint;
    vector<Point> points = {}; // �׷����� ��� ����
    vector<vector<Edge>> adj_list = {}; // ���� ����Ʈ

    YaoGraph(int n, int k) : pointCount(n), adj_list(n), coneCount(k) {} // ���� �� n���� �׷��� �ʱ�ȭ
};

// �� �� ������ �Ÿ��� ����ϴ� �Լ�
double distance(const Point& p1, const Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// �� ���� ��� ������ ����ϴ� �Լ� (0~360�� ������ ��)
double computeRelativeAngle(const Point& p1, const Point& p2) {
    double angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / M_PI;
    return angle >= 0 ? angle : 360.0 + angle; // ������ ��� ����� ��ȯ
}

// �� ������ ���̸� ����ϴ� �Լ� (0 ~ 360�� ������ ��)
double computeAngleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    if (diff > 360.0)
        diff -= 360.0;
    else if (diff < -180.0)
        diff += 360.0;
    return diff;
}

// shortest_paths[P] = 0���� �ʱ�ȭ�ϰ�, �������� ���Ѵ�� �����ϴ� �Լ�
void initializeShortestPaths(YaoGraph& G, unordered_map<int, double>& shortest_paths, int P) {
    shortest_paths.clear();
    for (int i = 0; i < G.pointCount; i++)
        shortest_paths[i] = 9999999;
    shortest_paths[P] = 0;
}

// Dijkstra �˰����� �̿��Ͽ� �ִ� ��� �� stretch factor�� ����ϴ� �Լ�
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
    int minPoint = -1;
    for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
        if (centerPoint == curPoint)
            continue;

        double relative_angle = computeRelativeAngle(G.points[centerPoint], G.points[curPoint]);

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

// Yao Graph�� ray�� ȸ���ϸ� stretch factor�� �Ź� ����ϴ� �Լ�
void rotateYaoGraph(YaoGraph& G, double rightRayAngle, double rotate) {
    double current_angle = rightRayAngle;
    unordered_map<int, double> shortest_paths;

    while (current_angle < rightRayAngle + 360 / G.coneCount) {
        // �ش� ���� ���� ������ PQ�� ����
        // memset(G.adjList, 0, sizeof(G.adjList));
        for (int point = 0; point < G.pointCount; point++)
            G.adj_list[point].clear();

        for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
            // vector<Edge> adjList;
            // ������ cone���� ���� ����� ���� edge�� �մ� �۾�
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

        if (current_angle == 0) {
            for (int i = 0; i < G.pointCount; i++) {
                cout << "\n\n***Cur Point: " << i << "***\n";
                for (auto it : G.adj_list[i])
                    cout << it.dest << " " << it.weight << "\n";
            }
        }

        G.stretchFactor = -1;
        for (int cur = 0; cur < G.pointCount; cur++) {
            shortest_paths.clear();
            shortest_paths = computeShortestPaths(G, cur, current_angle);
        }
        
        printf("Stretch Factor with reference angle %f: %f\n", current_angle, G.stretchFactor);

        printf("Points: %d, %d\n", G.factorPoint.first, G.factorPoint.second);

        printf("Coordinates: (%f, %f) and (%f, %f)\n", G.points[G.factorPoint.first].x, G.points[G.factorPoint.first].y, 
            G.points[G.factorPoint.second].x, G.points[G.factorPoint.second].y);

        printf("Euclidean Distance: %f\n\n\n", distance(G.points[G.factorPoint.first], G.points[G.factorPoint.second]));
        
        current_angle += rotate;
    }
}

int nearest(YaoGraph& G, int point) {
    double minDist = 999999;
    int nearPoint = -1;
    for (int cur = 0; cur < G.pointCount; cur++) {
        if (point == cur)
            continue;

        double dist = distance(G.points[cur], G.points[point]);
        if (dist < minDist) {
            minDist = dist;
            nearPoint = cur;
        }
    }

    return nearPoint;
}

void computeStretchFactor(YaoGraph& G, double midAngle, bool isMaximum) {

    // maximum stretch factor ���
    if (isMaximum) {
        for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
            // vector<Edge> adjList;
            // ������ cone���� ���� ����� ���� edge�� �մ� �۾�
            for (int rightAngle = midAngle; rightAngle < 360 + midAngle; rightAngle += 360 / G.coneCount) {
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

        // stretch factor �ʱ�ȭ
        G.stretchFactor = -1.0;

        // stretch factor ����
        for (int j = 0; j < G.pointCount; j++) {
            computeShortestPaths(G, j, true);
        }
    }
    // average stretch factor ���
    else {
        for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
            // vector<Edge> adjList;
            // ������ cone���� ���� ����� ���� edge�� �մ� �۾�
            for (int rightAngle = midAngle; rightAngle < 360 + midAngle; rightAngle += 360 / G.coneCount) {
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

        // stretch factor ����
        for (int j = 0; j < G.pointCount; j++) {
            computeShortestPaths(G, j, false);
        }

        // G.stretchFactor = G.stretchFactorSum / double(G.pairCount);
    }
}

int simulatedAnnealing(YaoGraph& G, vector<double> midAngles, bool isMaximum) {
    bool printFlag = false;

    // �ʱ���: �ƹ����Գ� ���
    int initID = rand() / midAngles.size();
    computeStretchFactor(G, midAngles[initID], true);
    double initSol = G.stretchFactor;

    // ������
    int  curID = initID;
    double curSol = initSol;

    int numIters = midAngles.size();
    int curIter = 0;

    double T0 = 10;
    double T = T0;

    double max = 32767;

    // �µ��� ����� ������ ������
    while (curIter < numIters) {
        T *= 0.95;
        curIter++;

        if (printFlag) cout << "T: " << T << ", " << "iter: " << curIter << endl;

        // �ĺ���: �ƹ����Գ� ���
        int candID = rand() / midAngles.size();
        computeStretchFactor(G, midAngles[candID], true);
        double candSol = G.stretchFactor;

        if (printFlag) cout << "candID: " << candID << ", " << "candSol: " << candSol; // << endl;

        // �ĺ���: �ִ� step (���� �پ��) �ȿ� �ִ� �ֵ� �߿��� ��������?
        // int candID

        // �ĺ��ذ� �ʱ��غ��� ���ٸ�, �ĺ��ظ� �����ط� ����
        if (candSol < curSol) {
            curID = candID;
            curSol = candSol;

            if (printFlag) cout << " (changed)";
        }
        // �ĺ��ذ� �ʱ��غ��� ���ڴٸ�, ������ Ȯ���� �����ظ� ����
        else {
            // x�� �ĺ��ؿ� �ʱ����� ����
            // �ּ�ȭ �����̹Ƿ� (�ʱ��� - �ĺ���)�� ���� ���ؾ� ��
            double x = (curSol - candSol) / T;
            double changeProb = exp(x);

            if (rand() / max < changeProb) {
                curSol = candSol;
                curID = candID;

                if (printFlag) cout << " (changed)";
            }
            else {
                if (printFlag) cout << " (unchanged)";
            }
        }

        if (printFlag) cout << endl;

        if (printFlag) cout << "curID: " << curID << ", " << "curSol: " << curSol << endl;
    }

    return curID;
}


int main(void) {
    srand(time(NULL));
    YaoGraph G(20, 7);
    double new_x, new_y;

    //G.points.push_back(Point(0.0, 0.0));
    //G.points.push_back(Point(1.0, 1.0));
    //G.points.push_back(Point(1.0, -1.0));
    //G.points.push_back(Point(-1.0, 1.0));
    //G.points.push_back(Point(-1.0, -1.0));

    mt19937 engine((unsigned int)time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    auto generator = bind(distribution, engine);

    for (int i = 0; i < G.pointCount; i++) {
        Point newPoint(rand() / 32767, rand() / 32767);
        G.points.push_back(newPoint);
    }

    /*
    for (int curPoint = 0; curPoint < G.pointCount; curPoint++) {
        vector<Edge> adjList;
        // ������ cone���� ���� ����� ���� edge�� �մ� �۾�
        for (int rightAngle = 0; rightAngle < 360; rightAngle += 360 / G.coneCount) {
            int destPoint = insert(G, curPoint, rightAngle);
            if (destPoint == -1)
                continue;

            double dist = distance(G.points[curPoint], G.points[destPoint]);
            Edge newEdge(destPoint, dist);
            adjList.push_back(newEdge);
        }
        
        G.adj_list[curPoint] = adjList;
    }
    */
    int start = clock();
    rotateYaoGraph(G, 0, M_PI / 12);
    cout << "\nTime taken: " << (double)(clock() - start) << '\n';
    return 0;
}