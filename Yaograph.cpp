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
#include <fstream>

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

    double stretchFactor = -1.0;
    double stretchFactorSum = 0.0;
    double pairCount = 0;

    vector<Point> points = {}; // 그래프의 모든 점들
    vector<vector<Edge>> adj_list = {}; // 인접 리스트
    vector<vector<Edge>> bestAdjList = {};
    // double averageStretchFactor = -1.0;

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
    void initialize();
    void partialInitialize();
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

void YaoGraph::initialize() {
    pointCount = 0;
    stretchFactor = -1.0;
    stretchFactorSum = 0.0;
    pairCount = 0;
    points = {}; 
    adj_list = {}; 
    bestAdjList = {};
    graphPath = {}; 
    directDistance = -1; 
    graphDistance = -1;
}

void YaoGraph::partialInitialize() {
    stretchFactor = -1.0;
    stretchFactorSum = 0.0;
    pairCount = 0;
    // adj_list = {};
    // bestAdjList = {};
    // graphPath = {};
    directDistance = -1;
    graphDistance = -1;
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
unordered_map<int, double> computeShortestPaths(YaoGraph& G, int P, bool isMaximum) {
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

                // compute the maximum stretch factor
                if (isMaximum) {
                    if (G.stretchFactor < graphDistance / directDistance) {
                        G.stretchFactor = graphDistance / directDistance;

                        G.bestAdjList = G.adj_list;

                        // graphPath, graphDistance, directDistance 업데이트
                        G.directDistance = directDistance;
                        G.graphDistance = graphDistance;
                        vector<int>({ next_point }).swap(G.graphPath);

                        int cur = next_point;

                        while (cur != prev[cur]) {
                            G.graphPath.insert(G.graphPath.begin(), prev[cur]);
                            cur = prev[cur];
                        }
                    }
                }
                // compute the average stretch factor
                else {
                    G.stretchFactorSum += graphDistance / directDistance;
                    G.pairCount += 1;
                }
            }
        }
    }

    return shortest_paths;
}

//// Yao Graph의 ray를 회전하며 stretch factor를 매번 계산하는 함수
//void rotateYaoGraph(YaoGraph& G, double rightRayAngle, double rotate) {
//    double current_angle = rightRayAngle;
//    unordered_map<int, double> shortest_paths;
//
//    while (current_angle < rightRayAngle + 360 / G.coneCount) {
//        // 해당 영역 안의 점들을 PQ에 삽입
//        G.targetPoints = {};
//        G.targets = {};
//        G.stretchFactor = -1;
//        for (int cur = 1; cur < G.pointCount; cur++) {
//            double relative_angle = computeRelativeAngle(G.points[0], G.points[cur]);
//            double angle_diff = relative_angle - current_angle;
//
//            if (angle_diff >= 0 && angle_diff < 360 / G.coneCount) {
//                G.targetPoints.push(make_pair(angle_diff, cur));
//                G.targets.push_back(cur);
//            }
//        }
//
//        for (int i = 0; i < G.targets.size(); i++)
//            shortest_paths = computeShortestPaths(G, G.targets[i], isMaximum);
//
//        printf("Stretch Factor with reference angle %f: %f\n", current_angle, G.stretchFactor);
//        current_angle += rotate;
//    }
//}

void computeYaoGraph(YaoGraph& G, double referenceAngle) {
    // adjacency list를 초기화
    for (auto& list : G.adj_list) vector<Edge>().swap(list);
    // average stretch factor를 초기화
    // G.averageStretchFactor = -1.0;

    int cone = G.coneCount;
    int pt = G.pointCount;
    double oneRound = 360.0 / double(cone);

    // 각 point마다
    for (int i = 0; i < pt; i++) {
        // k개의 cone 각각에 이웃 point 저장(각 group당 하나씩 선택됨)
        // pair는 (이웃 point의 index, 거리)로 이루어짐
        vector<vector<pair<double, int>>> conePoints = {};
        conePoints.assign(cone, {});

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
            Edge E2(i, group[0].first);
            G.adj_list[group[0].second].push_back(E2);

            if (i == group[0].second) {
                int _ = 0;
            }
        }
        
    }
    
}

vector<double> computeMidAngles(YaoGraph& G, double start, double end) {
    vector<double> angles;

    // pairwise angle 계산 후 sorting
    for (int i = 0; i < G.pointCount; i++) {
        for (int j = 0; j < G.pointCount; j++) {
            if (i == j) continue;

            double angle = computeRelativeAngle(G.points[i], G.points[j]);
            angles.push_back(angle);
        }
    }

    angles.push_back(start);
    angles.push_back(end);

    sort(angles.begin(), angles.end());

    vector<double> midAngles;
    for (int i = 0; i < angles.size() - 1; i++) {
        midAngles.push_back((angles[i] + angles[i + 1]) / 2);
    }

    return midAngles;
}


void computeStretchFactor(YaoGraph& G, double midAngle, bool isMaximum) {
    
    // maximum stretch factor 계산
    if (isMaximum) {
        computeYaoGraph(G, midAngle);

        // stretch factor 초기화
        G.stretchFactor = -1.0;

        // stretch factor 재계산
        for (int j = 0; j < G.pointCount; j++) {
            computeShortestPaths(G, j, true);
        }
    }
    // average stretch factor 계산
    else {
        computeYaoGraph(G, midAngle);

        // stretch factor 재계산
        for (int j = 0; j < G.pointCount; j++) {
            computeShortestPaths(G, j, false);
        }

        G.stretchFactor = G.stretchFactorSum / double(G.pairCount);
    }
}

// simulated annealing 같이 할 수 있도록?
// return type : bestID
int simulatedAnnealing(YaoGraph& G, vector<double> midAngles, bool isMaximum) {

    G.partialInitialize();

    bool printFlag = false;
    
    // 초기해: 아무렇게나 잡기
    int initID = rand() / midAngles.size();
    computeStretchFactor(G, midAngles[initID], true);
    double initSol = G.stretchFactor;
    
    // 현재해
    int  curID = initID;
    double curSol = initSol;

    int numIters = midAngles.size() / 3;
    int curIter = 0;

    double T0 = 10;
    double T = T0;

    double max = 32767;

    // 온도가 충분히 낮아질 때까지
    while (curIter < numIters) {
        T *= 1 - 0.05 * (double(midAngles.size()) / double(numIters));
        curIter++;

        if (printFlag) cout << "T: " << T << ", " << "iter: " << curIter << endl;

        // 후보해: 아무렇게나 잡기
        int candID = rand() / midAngles.size();
        computeStretchFactor(G, midAngles[candID], true);
        double candSol = G.stretchFactor;

        if (printFlag) cout << "candID: " << candID << ", " << "candSol: " << candSol; // << endl;

        // 후보해: 최대 step (점점 줄어듬) 안에 있는 애들 중에서 랜덤으로?
        // int candID

        // 후보해가 초기해보다 낫다면, 후보해를 다음해로 선택
        if (candSol < curSol) {
            curID = candID;
            curSol = candSol;

            if (printFlag) cout << " (changed)";
        }
        // 후보해가 초기해보다 나쁘다면, 다음의 확률로 다음해를 선택
        else {
            // x는 후보해와 초기해의 차이
            // 최소화 문제이므로 (초기해 - 후보해)로 값을 정해야 함
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

// local search

// Heuristic 1
// start - end 사이를 t개의 구간으로 나눔
// 그 중에서 제일 좋은 (양 끝 index에 대한 stretch factor의 평균이 가장 낮은) 구간 하나를 반환
tuple<double, int, int> randomSampling(YaoGraph& G, vector<double> midAngles, bool isMaximum, int start, int end, int t) {
    
    // start에서 시작해서
    // step마다 계속 띄엄띄엄 가게 됨
    double step = double(end - start) / double(t);

    // double curAngle = -1;

    vector<int> IDs = { start };
    double curID = start;
    
    while (true) {
        curID += step;

        if (curID > end) {
            IDs.push_back(end); break;
        }
        else {
            IDs.push_back(floor(curID));
        }
    }

    vector<tuple<double, int, int>> tupleVec;

    // stretch factor intervals
    vector<tuple<double, int, int>> intervals;
    for (int i = 0; i + 1 < IDs.size(); i++) {
        double angle1 = midAngles[IDs[i]];
        double angle2 = midAngles[IDs[i+1]];

        computeStretchFactor(G, angle1, isMaximum); double factor1 = G.stretchFactor;
        computeStretchFactor(G, angle2, isMaximum); double factor2 = G.stretchFactor;

        tupleVec.push_back(make_tuple((factor1 + factor2) / 2, IDs[i], IDs[i + 1]));
    }

    sort(tupleVec.begin(), tupleVec.end());

    return tupleVec[0];

}

// 가장 좋은 ID를 반환
int bruteForce(YaoGraph& G, vector<double> midAngles, int start, int end, bool isMaximum) {

    double bestID = -1;
    double minStretchFactor = DBL_MAX;

    for (int i = start; i <= end; i++) {
        computeStretchFactor(G, midAngles[i], isMaximum);
        if (minStretchFactor > G.stretchFactor) {
            minStretchFactor = G.stretchFactor;
            bestID = i;
        }
    }

    return bestID;
}

void generateRandomPoints(YaoGraph& G, int numPoints) {

    G.initialize();

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
}

// 성능 계산
// average stretch factor 계산
// 등수, 비율 모두 계산
int computePerformance(YaoGraph& G, vector<double> midAngles, int myID, int isMaximum) {
    cout << endl;

    double minStretchFactor = DBL_MAX; int minID = -1;
    double maxStretchFactor = DBL_MIN; int maxID = -1;
    
    // stretch factor, ID 순
    vector<pair<double, int>> pairVec;

    computeStretchFactor(G, midAngles[myID], isMaximum);
    double myStretchFactor = G.stretchFactor;

    for (int i = 0; i < midAngles.size(); i++) {
        auto& midAngle = midAngles[i];

        computeStretchFactor(G, midAngle, isMaximum);

        pairVec.push_back(make_pair(G.stretchFactor, i));

        if (minStretchFactor > G.stretchFactor) {
            minStretchFactor = G.stretchFactor;
            minID = i;
        }
        if (maxStretchFactor < G.stretchFactor) {
            maxStretchFactor = G.stretchFactor;
            maxID = i;
        }
    }

    sort(pairVec.begin(), pairVec.end());

    int rank = -1;
    // 등수 계산
    for (int i = 0; i < pairVec.size(); i++) {
        if (pairVec[i].second == myID) {
            // 공동 등수로 매김(앞으로 올 수 있는 만큼 이동)
            while (i - 1 >= 0 && pairVec[i].first == pairVec[i-1].first) {
                i--;
            }
            rank = i;
            break;
        }
    }

    cout << "Rank: " << rank << " out of " << pairVec.size() << endl;
    cout << endl;

    double maxMinDifference = maxStretchFactor - minStretchFactor;
    double myDifference = myStretchFactor - minStretchFactor;

    string s = "Average";
    if (isMaximum) s = "Maximum";

    cout << "Min " + s + " Stretch Factor : " << minStretchFactor << endl;
    cout << "Max " + s + " Stretch Factor: " << maxStretchFactor << endl;

    cout << "My " + s + " Stretch Factor: " << myStretchFactor << endl;
    // cout << "Max-Min Difference: " << maxMinDifference << endl;
    // cout << "My Difference: " << myDifference << endl;

    cout << "Approximation Factor: " << myStretchFactor / minStretchFactor << endl;
    cout << endl;

    return rank;
}

int main() {

    ofstream fout;
    // fout.open("C:\\Users\\hwikim\\Dropbox\\ALGOLAB\\연구참여\\신재욱학생\\points.txt");
    // fout.open("points.txt");

    // 랜덤 시드 설정
    // srand(time(NULL));
    // 
    // 
    bool PRINT_ADJ_LIST = true;

    bool ranDom = true;
    int numCones = 7;
    bool isMaximum = true;

    // point 10개, cone 3개인 YaoGraph 생성
    // YaoGraph G(10, 3);
    YaoGraph G(numCones);
    
    int randomNumPoints = 20;
    int numIterations = 5;

    clock_t start, finish;
    double result;

    vector<double> SATime;
    vector<int> SARank;
    vector<double> RSTime;
    vector<int> RSRank;
    vector<double> BFTime;
    vector<int> BFRank;

    int bestID;
    int rank;

    for (int i = 0; i < numIterations; i++) {
        generateRandomPoints(G, randomNumPoints);

        //fout << G.pointCount << endl;
        //for (int i = 0; i < G.pointCount; i++) {
        //    cout << "Point " << i << ": (" << G.points[i].x << ", " << G.points[i].y << ")" << endl;
        //    fout << G.points[i].x << " " << G.points[i].y << endl;
        //}
        //cout << endl;
        //fout << endl;

        //fout.close();

        //fout.open("C:\\Program Files\\gnuplot\\bin\\stretchFactor.txt");

        double start = 0.0; double end = 360.0 / double(G.coneCount);
        vector<double> midAngles = computeMidAngles(G, start, end);

        start = clock();

        bestID = bruteForce(G, midAngles, 0, midAngles.size()-1, isMaximum);

        end = clock();

        rank = computePerformance(G, midAngles, bestID, isMaximum);

        result = (double)(end - start);

        cout << "time taken: " << result / CLOCKS_PER_SEC << "second" << endl;

        BFTime.push_back(result);
        BFRank.push_back(rank);

        start = clock();

        // 나누는 개수(t)보다는 base_size가 더 커야 함.
        int t = 5;
        int base_size = 10;
        // int base_size = 3;

        int startID = 0;
        int endID = midAngles.size() - 1;

        while (endID - startID > base_size) {
            auto trp = randomSampling(G, midAngles, isMaximum, startID, endID, 5);
            startID = get<1>(trp);
            endID = get<2>(trp);
        }

        // 남은 경우들은 다 직접 계산하고 그 중에서 가장 좋은 ID를 찾음.
        bestID = bruteForce(G, midAngles, startID, endID, isMaximum);

        end = clock();

        rank = computePerformance(G, midAngles, bestID, isMaximum);

        result = (double)(end - start);

        cout << "time taken: " << result / CLOCKS_PER_SEC << "second" << endl;

        RSTime.push_back(result);
        RSRank.push_back(rank);

        start = clock();

        bestID = simulatedAnnealing(G, midAngles, true);

        end = clock();

        rank = computePerformance(G, midAngles, bestID, isMaximum);

		result = (double)(end-start);

		cout << "time taken: " << result / CLOCKS_PER_SEC << "second" << endl;

        SATime.push_back(result);
        SARank.push_back(rank);
    }

    fout.open("C:\\Users\\hwikim\\Desktop\\result.txt");

    double BFavgTime = 0;
    for (auto t : BFTime) {
        fout << t / 1000 << " ";
        BFavgTime += t / 1000;
    }
    fout << endl;
    fout << BFavgTime / numIterations << endl;

    for (auto t : BFRank) fout << t << " ";
    fout << endl;

    double RSavgTime = 0;
    for (auto t : RSTime) {
        fout << t / 1000 << " ";
        RSavgTime += t / 1000;
    }
    fout << endl;
    fout << RSavgTime / numIterations << endl;

    for (auto t : RSRank) fout << t << " ";
    fout << endl;

    double SAavgTime = 0;
    for (auto t : SATime) {
        fout << t / 1000 << " ";
        SAavgTime += t / 1000;
    }
    fout << endl;
    fout << SAavgTime / numIterations << endl;

    for (auto t : SARank) fout << t << " ";
    fout << endl;

    fout.close();
}

//if (ranDom) {
  //    generateRandomPoints(G, randomNumPoints);
  //}
  //else {
  //    G.addPoint(Point(0.0, 0.0));
  //    G.addPoint(Point(1.0, 1.0));
  //    G.addPoint(Point(1.0, -1.0));
  //    G.addPoint(Point(-1.0, -1.0));
  //    G.addPoint(Point(-1.0, 1.0));
  //}

  //int totalChangeNum = 0;
  //int changeNum = 0;
  //double prevFactor = -1;
  //// double curFactor = -1;
  //double eps = 0.001;

  //bool first = true;

  //for (auto& midAngle : midAngles) {
  //    if (midAngle >= end) continue;

  //    totalChangeNum++;

  //    computeStretchFactor(G, midAngle);
  //    
  //    if (prevFactor == -1) prevFactor = G.stretchFactor;
  //    else if (abs(prevFactor - G.stretchFactor) > eps) {
  //        prevFactor = G.stretchFactor;
  //        changeNum++;
  //    }

  //    printf("Stretch Factor with reference angle %f: %f\n", midAngle, G.stretchFactor);

  //    fout << midAngle << " " << G.stretchFactor << endl;

  //    if (!G.graphPath.empty()) {

  //        if (PRINT_ADJ_LIST && first) {
  //            cout << "Adjacency List" << endl;
  //            for (int k = 0; k < G.pointCount; k++) {
  //                auto& row = G.bestAdjList[k];

  //                for (int ii = 0; ii < G.pointCount; ii++) {
  //                    bool exist = false;
  //                    for (auto& jj : row) {
  //                        if (jj.dest == ii) {
  //                            cout << "1 ";
  //                            exist = true;
  //                            break;
  //                        }
  //                    }
  //                    if (!exist) cout << "0 ";
  //                }
  //                cout << endl;
  //            }
  //            first = false;
  //            // cout << endl;
  //        }

  //        cout << "Worst Pair: (" << G.graphPath[0] << ", " << G.graphPath[G.graphPath.size() - 1] << ")" << endl;

  //        int i = 0;
  //        cout << "Graph Path: ";
  //        while (true) {
  //            cout << G.graphPath[i] << " ";
  //            i++;
  //            if (i >= G.graphPath.size()) break;
  //        }
  //        cout << endl;

  //        i = 0;
  //        cout << "Graph Path Edge Length: ";
  //        while (i + 1 < G.graphPath.size()) {
  //            cout << distance(G.points[G.graphPath[i]], G.points[G.graphPath[i + 1]]) << " ";
  //            i++;
  //        }
  //        cout << endl;

  //        cout << "Graph Distance : " << G.graphDistance << endl;

  //        cout << "Direct Distance: " << G.directDistance;

  //        // cout << ", On-site Computation: " << distance(G.points[G.graphPath[0]], G.points[G.graphPath[G.graphPath.size() - 1]]) << endl;
  //        cout << endl;
  //    }

  //    cout << endl;
  //}

  //fout.close();

  //cout << "Total stretch factor changes: " << changeNum << " out of " << totalChangeNum << endl;

  // computePerformance(G, midAngles, 0);
  // computePerformance(G, midAngles, 5);