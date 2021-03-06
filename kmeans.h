//
// Created by mitom on 8/9/21.
//

#ifndef BMEA_KMEANS_H
#define BMEA_KMEANS_H

#include <math.h>
#include "kmeans_oop.h"
#include "./geometry/PointCloud.h"
#include "./util/string_util.h"
#include "./util/mathop.h"
using std::cin;
using std::cout;
using std::initializer_list;
using std::runtime_error;
class NDimenPoint : public VirtualPoint {
private:
    int dimension;
    vector<double> xs;
public:
    NDimenPoint(const int d) : dimension(d) { xs.resize(d); }
    NDimenPoint(const int d, vector<double> l) : dimension(d), xs(l){};
    NDimenPoint(const NDimenPoint &p) : dimension(p.dimension), xs(p.xs) {}
    ~NDimenPoint(){};
    bool operator==(const VirtualPoint &p) override {
        auto pp = static_cast<const NDimenPoint &>(p);
        if (dimension != pp.dimension) return false;
        for (size_t i = 0; i < xs.size(); i++)
            if (xs[i] != pp.xs[i]) return false;
        return true;
    }
    bool operator!=(const VirtualPoint &p) override {
        auto pp = static_cast<const NDimenPoint &>(p);
        if (dimension != pp.dimension) return true;
        for (size_t i = 0; i < xs.size(); i++)
            if (xs[i] != pp.xs[i]) return true;
        return false;
    }
    void add(const NDimenPoint &p) {
        if (p.dimension != dimension) throw runtime_error("dimension mismatch");
        for (size_t i = 0; i < xs.size(); i++)
            xs[i] += p.xs[i];
    }
    NDimenPoint operator/(const int n) {
        if (n == 0) throw std::runtime_error("divisor zero error!");
        NDimenPoint res(dimension);
        for (size_t i = 0; i < dimension; i++) {
            res.xs[i] = xs[i] / n;
        }
        return res;
    }
    double disTo(const NDimenPoint &p) {
        double tmp = 0;
        for (size_t i = 0; i < dimension; i++) tmp += pow(xs[i] - p.xs[i], 2);
        return sqrt(tmp);
    }
    string toString() override {
        stringstream ss;
        ss << "[";
        for (size_t i = 0; i < dimension; i++) {
            if (i > 0) ss << ", ";
            ss << xs[i];
        }
        ss << "]";
        return ss.str();
    }

    /**
     * ??????????????????????????????????????????double??????
     * @return
     */
    double toDouble() override {
        vector<double> point = getPoint();
        return point[0];
    }

    vector<double> getPoint() const
    {
        return this->xs;
    }

    static double calcDisToCluster(const VirtualPoint &p, const Cluster &c) {
        auto pp = static_cast<const NDimenPoint &>(p);
        auto cp = static_cast<const NDimenPoint &>(*(c.getCentroid()));
        return pp.disTo(cp);
    }
    static sharedVPoint avgPoints(const vector<sharedVPoint> &points) {
        if (points.size() <= 0) return nullptr;
        NDimenPoint resPoint(static_cast<const NDimenPoint &>(*points[0]).dimension);
        for (auto &&p : points)
            resPoint.add(static_cast<const NDimenPoint &>(*p));
        resPoint = resPoint / points.size();
        // cerr << "DEBUG\t" << resPoint.toString() << ", POINTS.SIZE " << points.size() << endl;
        return make_shared<NDimenPoint>(resPoint);
    };
};
vector<NDimenPoint> geneData(int num, const int dimension, double maxVal = 1000) {
    std::default_random_engine generator(time(NULL));
    std::uniform_real_distribution<double> distribution(0, maxVal);
    vector<NDimenPoint> points;
    for (size_t i = 0; i < num; i++) {
        vector<double> tmpVec;
        for (size_t j = 0; j < dimension; j++)
            tmpVec.push_back(distribution(generator));
        points.push_back(NDimenPoint(dimension, tmpVec));
    }
    return points;
}

vector<NDimenPoint> initData(int dimension, geometry::PointCloud pc)
{
    vector<NDimenPoint> points;
    vector<double> zcol = pc.getZCol();
    for(size_t i=0; i<zcol.size(); i++){
        vector<double> tmpVec;
        for (size_t j = 0; j < dimension; j++){
            tmpVec.push_back(zcol[i]);
        }
        points.push_back(NDimenPoint(dimension, tmpVec));
    }
    return points;
}



void output(const vector<Cluster> &clusters, const int dimension) {
    cout << "{"
         << "\"dimension\":" << dimension << "," << endl
         << "\"clusters\":[";
    for (int i = 0; i < clusters.size(); i++) {
        if (i > 0) cout << ", ";
        std::cout << clusters[i].toString() << std::endl;
    }
    cout << "]}" << endl;
}



/**
 *  Kmean??????????????????
 * @param pc ??????(?????????)
 * @param k  ?????????
 * @return ????????????
 */
vector<Cluster> kmeans_work(geometry::PointCloud pc, int k) {
    const int maxRound = 10000; // ???????????????
    const int pointCnt = 10;  // ?????????
    int dimension = 1;
    vector<sharedVPoint> points;
    for (auto &&p : initData(1, pc)) {
        points.push_back(make_shared<NDimenPoint>(p));
    }

    auto clusters = KmeansAlg::run(points, k, NDimenPoint::calcDisToCluster, NDimenPoint::avgPoints, maxRound);
//    output(clusters, dimension);

//    for (auto cluster : clusters){
//        vector<sharedVPoint> points = cluster.getPoints();
//        for (auto p : points){
//             cout << p->toDouble()<< " ";
//        }
//        cout << endl;
//    }
    return clusters;
}

/**
 * ??????????????????
 * @param pc
 * @param clusters
 * @return
 */
std::vector<int> kmeansPredict(const geometry::PointCloud& pc, vector<Cluster> clusters)
{
    vector<double> zcol = pc.getZCol();
    vector<int> predictIndex(zcol.size());
    for(int i=0; i<clusters.size(); i++){
        vector<sharedVPoint> points = clusters[i].getPoints();
        for(auto point : points){
            for(int j=0; j<zcol.size(); j++){
                if(point->toDouble() == zcol[j]) predictIndex[j] = i;
            }
        }
    }
    return predictIndex;
}

/**
 * ??????????????????
 * https://blog.csdn.net/wangxiaopeng0329/article/details/53542606
 * https://zhuanlan.zhihu.com/p/108163834
 * @param pc
 * @param predictIndex
 * @param k
 * @return
 */
double calcPerformance(const geometry::PointCloud& pc, const std::vector<int>& predictIndex, int k)
{
    vector<double> zcol = pc.getZCol();
    // ??????????????????
    vector<vector<int>> clusterIndexs;
    // ????????????a b ???
    vector<double> a_i(predictIndex.size());
    vector<double> b_i(predictIndex.size());

    // ?????????????????????
    for (int i=0; i<k; i++){
        vector<int> sameClusterIndex;
        for (int j=0; j<predictIndex.size(); j++){
            if(predictIndex[j] == i)  sameClusterIndex.push_back(j);
        }
        // ??????????????????????????????????????????????????????
        if(sameClusterIndex.size() <= 4) {
            return -1;
        }
        clusterIndexs.push_back(sameClusterIndex);
    }

    // a_i
    for (int i=0; i<k; i++){
        // ????????????????????????????????????????????????
        vector<int> clusterIndex = clusterIndexs[i];
        for (int j=0; j<clusterIndex.size(); j++){
            double distance = 0;
            for (int m=0; m<clusterIndex.size(); m++) {
                distance += abs(zcol[clusterIndex[j]] - zcol[clusterIndex[m]]);
            }
            double avg_distance = distance * 1.0 / (clusterIndex.size() - 1);
            // ??????
            a_i[clusterIndex[j]] = avg_distance;
        }
    }

    // b_i
    int kt = k;
    for(int i=0; i<k; i++){
        vector<int> clusterIndex1 = clusterIndexs[i]; // ??????A
        for (int j=0; j<clusterIndex1.size(); j++){ // A???????????? j
            double min_distence = 100000000; // ???????????????????????????
            for (int m=0; m<kt; m++){ // ?????????A?????????????????????
                if(i==m)  continue;
                vector<int> clusterIndex2 = clusterIndexs[m];
                double other_distence = 0;
                for (int n=0; n<clusterIndex2.size(); n++){  // n
                    other_distence += abs(zcol[clusterIndex1[j]] - zcol[clusterIndex2[n]]);
                }
                double avg_distance = other_distence * 1.0 / (clusterIndex2.size());
                if(avg_distance < min_distence){ // ???????????????????????????????????????????????????????????????????????????????????????
                    min_distence = avg_distance;
                }
            }
            b_i[clusterIndex1[j]] = min_distence;
        }
    }
    // ?????????????????? s
    double s = 0;
    for (int i=0; i<a_i.size(); i++){
       s += (b_i[i] - a_i[i]) / (b_i[i] > a_i[i] ? b_i[i] : a_i[i]); // s = (b-a)/max(a,b)
    }
    return s / a_i.size();
}

/**
 * ?????????????????????Clusters
 * @param pc
 * @return
 */
vector<Cluster> kmeans(const geometry::PointCloud& pc)
{
    int k = 0;
    double performance = -1;
    vector<double> pf_v;
    vector<vector<Cluster>> clusters_k;
    for (int i=2; i<6; i++){
        vector<Cluster> clusters = kmeans_work(pc, i);
        clusters_k.push_back(clusters);
        std::vector<int> predictIndex = kmeansPredict(pc, clusters);
        performance = calcPerformance(pc, predictIndex, i);
        //cout << "performance==== " << performance << endl;
        pf_v.push_back(performance);
    }
    // ??????perfermance?????????k
    k = util::argmax(pf_v);
//    k = 4;
    return clusters_k[k];
}


#endif //BMEA_KMEANS_H