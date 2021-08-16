//
// Created by mitom on 8/10/21.
//

#ifndef BMEA_POINTCLOUD_H
#define BMEA_POINTCLOUD_H

#include <iostream>
#include <vector>
#include <fstream>
#include <assert.h>

#include "defines.h"
#include "Point.h"
#include "../util/string_util.h"

GEOMETRY_NAMESPACE_BEGIN

class PointCloud{
public:
    PointCloud(void) = default;
    PointCloud(const std::string& objfile);

    PointCloud& addPoint(double x, double y, double z);
    PointCloud& addPoint(const Point & point);
    std::vector<double> getXCol() const;
    std::vector<double> getYCol() const;
    std::vector<double> getZCol() const;

    int size() const;
    std::vector<Point> getPointSet() const;
    /** 对象操作 */
    Point& operator[] (int index);
    Point const& operator[] (int index) const;
    Point& operator() (int index);
    Point const& operator() (int index) const;

    void print();

protected:
    typedef std::vector<Point> PointSet;
    PointSet pointset;
};

PointCloud::PointCloud(const std::string& objfile) {
    std::ifstream ifs;
    ifs.open(objfile.data());
    assert(ifs.is_open());

    std::string s;
    while (getline(ifs, s)){
        if(util::contain(s, "v")){
            std::vector<std::string> items = util::boostsplit(s, " ");
            geometry::Point point(util::stod(items[1]), util::stod(items[2]), util::stod(items[3]));
            this->pointset.push_back(point);
        }
    }
    ifs.close();
}

PointCloud& PointCloud::addPoint(double x, double y, double z)
{
    this->pointset.push_back(Point(x, y, z));
    return *this;
}

PointCloud& PointCloud::addPoint(const Point & point)
{
    this->pointset.push_back(point);
    return *this;
}

std::vector<double> PointCloud::getXCol() const
{
    std::vector<double> xcol;
    for(auto point : pointset){
        xcol.push_back(point.getX());
    }
    return xcol;
}

std::vector<double> PointCloud::getYCol() const
{
    std::vector<double> ycol;
    for(auto point : pointset){
        ycol.push_back(point.getY());
    }
    return ycol;
}

std::vector<double> PointCloud::getZCol() const
{
    std::vector<double> zcol;
    for(auto point : pointset){
        zcol.push_back(point.getZ());
    }
    return zcol;
}

int PointCloud::size() const {
    return pointset.size();
}

std::vector<Point> PointCloud::getPointSet() const
{
    return this->pointset;
}

Point& PointCloud::operator[] (int index)
{
    return this->pointset[index];
}

Point const& PointCloud::operator[] (int index) const
{
    return this->pointset[index];
}

Point& PointCloud::operator() (int index)
{
    return this->pointset[index];
}

Point const& PointCloud::operator() (int index) const
{
    return this->pointset[index];
}

void PointCloud::print(){
    std::cout << "== point cloud ==" << std::endl;
    for(int i=0; i<this->pointset.size(); i++){
        std::cout << "Point[" << i << "]: (" << pointset[i].getX()
        << "," << pointset[i].getY()  << "," << pointset[i].getZ() << ")" << std::endl;
    }
}

GEOMETRY_NAMESPACE_END

#endif //BMEA_POINTCLOUD_H
