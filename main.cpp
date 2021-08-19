#include <iostream>
#include <vector>
#include <string>
#include "kmeans.h"
#include "geometry/Point.h"
#include "geometry/PointCloud.h"
#include "geometry/Plane.h"
#include "geometry/PlaneSet.h"
#include "extract_cad.h"

#include<opencv2/opencv.hpp>
#include </usr/include/python3.6/Python.h>
//#include "Extract.h"

using std::cout;
using std::endl;

/**
 * 调用python的sklearn. kmeans聚类算法对obj文件聚类并写入obj高度
 */
bool refineFile(char* objfile)
{
    Py_InitializeEx(1);
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/mitom/Music/BMEA')");

    //导入模块
    PyObject* pModule = PyImport_ImportModule("kmeans");

    if (!pModule){
        std::cout << "Python get kmeans module failed." << std::endl;
        return false;
    }
    std::cout << "Python get module succeed." << std::endl;

    PyObject* pfun = PyObject_GetAttrString(pModule, "run");

    PyObject *pArgs = PyTuple_New(1);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", objfile));

    if (!pfun || !PyCallable_Check(pfun)){
        std::cout << "Can't find funftion (run)" << std::endl;
        return false;
    }
    PyObject *pReturn = PyEval_CallObject(pfun, pArgs);
    if (pReturn == NULL){
        std::cout << "返回值出错" << std::endl;
        return false;
    }
    int nValue = -2;
    PyArg_Parse(pReturn, "i", &nValue);
    std::cout << "python文件生成状态： " << nValue << std::endl;

    Py_Finalize();
    std::cout << "==============================" << std::endl;
    return true;
}

void run(const std::string& filename){
    //std::string filename = "/home/mitom/data/obj/single-plat-result.obj";
    geometry::PointCloud pc(filename);
    cout << "点数：" << pc.size() << endl;
    std::vector<double> height_v = pc.getZCol();
    util::doUnique_d(height_v);
    std::sort(height_v.begin(), height_v.end(), util::cmp_sort);
    std::cout << "聚类数： " << height_v.size() << std::endl;
    for (auto h : height_v){
        std::cout << h << " ";
    }
    cout << endl;
    // 面
    geometry::PlaneSet ps(filename);
    std::cout << "面数：" << ps.size() << std::endl;

    // 返回最终contour
    geometry::Contour resultContour = extract(height_v, pc, ps);


    double rotation = getBuildingRotation(resultContour, pc);

    std::cout << "建筑偏转角: " << rotation << std::endl;
}

void test()
{
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
    char* filename = "/home/mitom/data/obj/single-plat-result.obj";
    bool rf = refineFile(filename);
    if(!rf){
        std::cout << "聚类模块错误!" << std::endl;
        return 0;
    }
    run("/home/mitom/data/obj/single-plat-result_refine_d.obj");


    return 0;
}
