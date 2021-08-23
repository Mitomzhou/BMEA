#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include "kmeans.h"
#include "geometry/Point.h"
#include "geometry/PointCloud.h"
#include "geometry/Plane.h"
#include "geometry/PlaneSet.h"
#include "extract_cad.h"

#include<opencv2/opencv.hpp>
#include </usr/include/python3.8/Python.h>
//#include "Extract.h"

using std::cout;
using std::endl;

/**
 * 调用python的sklearn. kmeans聚类算法对obj文件聚类并写入obj高度
 */
bool refineFile(char* objfile, const double& radian, int direction)
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

    // 参数解释：https://blog.csdn.net/mkc1989/article/details/38943927
    PyObject *pArgs = PyTuple_New(3);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", objfile));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", radian));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("i", direction));

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
    std::cout << "CAD: " << std::endl;
    resultContour.print();
    draw_contour(resultContour, pc);

    double rotation = getBuildingRotation(resultContour, pc);

    std::cout << "建筑偏转角: " << rotation << std::endl;
    //pc.print();
}

double getRotation()
{
    double rotation = 0;
    char* filename = "/home/mitom/data/obj/single-plat-result.obj";
    bool rf = refineFile(filename, rotation, 0);
    if(!rf){
        std::cout << "聚类模块错误!" << std::endl;
        return 0;
    }
    cout << rotation << endl;
    filename = "/home/mitom/data/obj/single-plat-result_d.obj";
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
    std::cout << "CAD: " << std::endl;
    resultContour.print();
    draw_contour(resultContour, pc);
    rotation = getBuildingRotation(resultContour, pc);
    std::cout << "建筑偏转角: " << rotation << std::endl;
    return rotation;
}

void test()
{
    std::string sfilename = "/home/mitom/data/obj/single-plat-result.obj";
    std::vector<geometry::Contour> CAD5;
    std::vector<std::string> direction_v = {"d","f","b","l","r"};
    //==============1、获取building的偏转角度====================
    double rotation = getRotation();
    //==============2、for 5 direction =================
    for(int i=0; i<1; i++){
        if(i == 0){
            char* cfilename = const_cast<char*>(sfilename.c_str());
            bool rf = refineFile(cfilename, rotation, i);
            if(!rf){
                std::cout << "聚类模块错误!" << std::endl;
                return ;
            }
            run(util::addtoFilename(sfilename, direction_v[0]));
        }else{
            // TODO
            std::string sfilename_ = util::addtoFilename(sfilename, direction_v[0]);
            char* cfilename_ = const_cast<char*>(sfilename_.c_str());
            bool rf = refineFile(cfilename_, rotation, i);
            if(!rf){
                std::cout << "聚类模块错误!" << std::endl;
                return ;
            }
        }
    }

    return ;
}

int main() {
//    char* filename = "/home/mitom/data/obj/single-plat-result_refine_new1.obj";
//    double radian = 0;
//    bool rf = refineFile(filename, radian); // 0.22285433253600082
//    if(!rf){
//        std::cout << "聚类模块错误!" << std::endl;
//        return 0;
//    }
//    run("/home/mitom/data/obj/single-plat-result_refine_new1_refine_d.obj");
    test();
    //cout << util::addtoFilename("/home/mitom/data/obj/single-plat-result.obj","d") << endl;
    return 0;
}
