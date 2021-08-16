//
// Created by mitom on 8/9/21.
//

#ifndef BMEA_KMEANS_OOP_H
#define BMEA_KMEANS_OOP_H

#include <algorithm>
#include <cmath>
#include <ctime>
#include <exception>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using std::cerr;
using std::endl;
using std::make_shared;
using std::pow;
using std::shared_ptr;
using std::sqrt;
using std::string;
using std::stringstream;
using std::to_string;
using std::vector;

/**
 * kmeans - 点作为数据，cluster是点的聚簇
 * BEGIN
 *      选出来 k 个点作为中心点生成聚簇
 *      循环
 *           计算点与聚簇的距离
 *           每个点加入到距离最近的聚簇中
 *           更新聚簇中心点
 *           聚簇中心点未变？退出
 *      输出聚簇
 * END
 *
 * 数据结构
 * 点 - ==() toString()
 * 聚簇 - 计算中心点()
 * calcDis(point cluster)
 * kmeans() -
*/

class VirtualPoint {
private:
public:
    VirtualPoint() {}
    virtual ~VirtualPoint() {}
    virtual bool operator==(const VirtualPoint &p) = 0;
    virtual bool operator!=(const VirtualPoint &p) = 0;
    virtual string toString() = 0;
    virtual double toDouble() = 0; // for point
};

typedef shared_ptr<VirtualPoint> sharedVPoint;
typedef sharedVPoint avgPointFunc(const vector<sharedVPoint> &);

class Cluster {
private:
    vector<sharedVPoint> points;
    sharedVPoint centroid;
    avgPointFunc *avgPoints;

public:
    Cluster(avgPointFunc avg) { avgPoints = avg; }
    ~Cluster() {}
    Cluster &setCentroid(sharedVPoint p) {
        centroid = p;
        points.push_back(p);
        return *this;
    }
    bool updateCentroid() {
        sharedVPoint tmpPoint = avgPoints(points);
        if (tmpPoint == nullptr) return false;
        bool changed;
        if (tmpPoint != nullptr && centroid != nullptr)
            changed = (*tmpPoint) != (*centroid);
        else
            changed = true;
        centroid = tmpPoint;
        return changed;
    }
    void clear() { points.clear(); }
    void addPoint(sharedVPoint p) {
        points.push_back(p);
    }
    string toString() const {
        stringstream ss;
        if (centroid == nullptr || points.size() == 0) return "{}";
        ss << "{\"centroid\": " << centroid->toString() << ",\"points\": [";
        for (int i = 0; i < points.size(); i++) {
            if (i > 0) ss << ", ";
            ss << points[i]->toString();
        }
        ss << "]}";
        return ss.str();
    }

    sharedVPoint getCentroid() const { return centroid; }
    const vector<sharedVPoint> &getPoints() { return points; }
};

// 计算 VirtualPoint 与 Cluster的质心 之间的距离
typedef double calcFunc(const VirtualPoint &, const Cluster &);

class KmeansAlg {
public:
    KmeansAlg() {}
    ~KmeansAlg() {}
    // 生成 k 个 位于 [0, n) 中的随机数, n < 100000000
    static vector<int> randDiffNumbers(int n, int k) {
        const int maxn = 100000000;
        vector<int> res;
        if (n <= 0 || n >= maxn)
            throw std::runtime_error("n is less than zero or greater than maxn(100,000,000)");
        for (int i = 0; i < n; i++)
            res.push_back(i);
        random_shuffle(res.begin(), res.end());
        res.resize(k);
        return res;
    }
    static vector<Cluster> run(vector<sharedVPoint> data, int k, calcFunc calcDis, avgPointFunc avgPoints, const int maxRuond = 10000) {
        if (k <= 1) throw std::runtime_error("k is less than 1");
        vector<Cluster> clusters;
        for (auto &&i : randDiffNumbers(data.size(), k))
            clusters.push_back(Cluster(avgPoints).setCentroid(data[i]));
        for (int round = 0; round < maxRuond; round++) {
            // 清空
            for (auto &&c : clusters) c.clear();
            for (size_t i = 0; i < data.size(); i++) {
                // 计算距离，加入到最近聚簇中
                double minDis = calcDis(*(data[i]), clusters[0]);
                int minIndex = 0;
                for (size_t j = 1; j < clusters.size(); j++) {
                    double tmpDis = calcDis(*(data[i]), clusters[j]);
                    if (tmpDis < minDis) minDis = tmpDis, minIndex = j;
                }
                clusters[minIndex].addPoint(data[i]);
            }
            bool changed = false;
            for (auto &&c : clusters) changed = changed || c.updateCentroid();
            if (!changed) break;
        }
        return clusters;
    }
};

#endif //BMEA_KMEANS_OOP_H
