#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "kmeans.h"
#include "geometry/PointCloud.h"
#include "geometry/Plane.h"
#include "geometry/PlaneSet.h"
#include "extract_cad.h"
#include "dxftest.h"

#include<opencv2/opencv.hpp>
#include </usr/include/python3.6/Python.h>

using std::cout;
using std::endl;

/**
 * 调用python的sklearn. kmeans聚类算法对obj文件聚类并写入obj高度
 */
bool refineFile(const char* objfile)
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

    PyObject* pfun = PyObject_GetAttrString(pModule, "main");
    // 参数解释：https://blog.csdn.net/mkc1989/article/details/38943927
    PyObject *pArgs = PyTuple_New(1);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", objfile));

    if (!pfun || !PyCallable_Check(pfun)){
        std::cout << "Can't find funftion (main)" << std::endl;
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
    geometry::PointCloud pc(filename);
    // cout << "点数：" << pc.size() << endl;
    std::vector<double> height_v = pc.getZCol();
    util::doUnique_d(height_v);
    std::sort(height_v.begin(), height_v.end(), util::cmp_sort);
    std::cout << "聚类数： " << height_v.size() << std::endl;
    for (auto h : height_v){
        std::cout << h << " ";
    }
    cout << endl;
    geometry::PlaneSet ps(filename);
    // std::cout << "面数：" << ps.size() << std::endl;

    // 返回最终contour
    geometry::Contour resultContour = extract(height_v, pc, ps);
    std::cout << "CAD: " << std::endl;
    resultContour.print();
    draw_contour(resultContour, pc);
    geometry::CADLinkSet cadlinkset = convertCoodsCADLink(resultContour, pc);
    drawDXF("/home/mitom/single_d.dxf", cadlinkset);
}

int main() {
    const char* filename = "/home/mitom/data/obj/single-plat-result.obj";
    bool rf = refineFile(filename);
    if(!rf){
        std::cout << "聚类模块错误!" << std::endl;
        return 0;
    }
    run("/home/mitom/data/obj/single-plat-result_s_d.obj");  // 3
//    run("/home/mitom/data/obj/single-plat-result_s_f.obj"); // 3  y+500
//    run("/home/mitom/data/obj/single-plat-result_s_b.obj"); // 3    存在一些问题
//    run("/home/mitom/data/obj/single-plat-result_s_l.obj");  // 3 x+500
//    run("/home/mitom/data/obj/single-plat-result_s_r.obj");  // 3 x+500

    return 0;
}
