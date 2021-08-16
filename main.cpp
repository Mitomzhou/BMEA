#include <iostream>
#include <vector>
#include <string>
#include "kmeans.h"
#include "geometry/Point.h"
#include "geometry/PointCloud.h"
#include "geometry/Plane.h"
#include "geometry/PlaneSet.h"
#include "extract_cad.h"
//#include "Extract.h"

using std::cout;
using std::endl;


void run(){
    std::string filename = "/home/mitom/data/obj/single-plat-result.obj";
    // 点
    geometry::PointCloud pc(filename);
    cout << "点数：" << pc.size() << endl;
    std::vector<Cluster> clusters = kmeans(pc);
    std::vector<double> height_v;
    for(auto cluster : clusters){
        double height = util::setPrecision(cluster.getCentroid()->toDouble(), 3);
        height_v.push_back(height);
        cout << height << " ";
    }
    cout << endl;

    // 面
    geometry::PlaneSet ps(filename);
    std::cout << "面数：" << ps.size() << std::endl;

    //ps.print();

    // 替换所有高度 height_v
    std::vector<int> predictIndex = kmeansPredict(pc, clusters);
    for(int i=0; i<pc.getPointSet().size(); i++){
        pc[i].setZ(height_v[predictIndex[i]]);
    }
    std::sort(height_v.begin(), height_v.end(), util::cmp_sort);
    extract(height_v, pc, ps);
//    ps.setPointIndex(89,0,88888888);
//    ps.print();
}

void test()
{
//    std::vector<int> v = {9,6,7,2,3,4,5,2,3,4,5};
////    util::doUnique(v);
////    for (auto a : v) std::cout << a << " ";
//
//    std::sort(v.begin(), v.end(), util::cmp_sort);
//
//    v.insert(v.begin(), 10000);
//    for (auto a : v) std::cout << a << " ";
//    cout << util::contain_i(v, 9);

    std::vector<std::pair<int, int>> vp;
    vp.push_back(std::make_pair(10,100));
    for(auto p : vp){
        p.first = 1000000;
    }
    for(auto p : vp){
        cout << p.first << endl;
    }

}

int main() {
     run();
//    test();

    return 0;
}
