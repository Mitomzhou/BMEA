//
// Created by mitom on 8/13/21.
//

#ifndef BMEA_PLANESET_H
#define BMEA_PLANESET_H


#include <iostream>
#include <vector>
#include <fstream>
#include <assert.h>

#include "defines.h"
#include "Plane.h"
#include "../util/string_util.h"

GEOMETRY_NAMESPACE_BEGIN

class PlaneSet{
public:
    PlaneSet(void) = default;
    PlaneSet(const std::string& objfile);

    PlaneSet& addPlane(std::vector<int> pointsIndex);
    PlaneSet& addPlane(const Plane & plane);

    std::vector<Plane> getPlaneSet();

    int size() const;

    /* 设置planeset 中某面某个点index */
    void setPointIndex(int planeIndex, int pointIndex, int index);
    int getPointIndex(int planeIndex, int pointIndex) const;

    /** 对象操作 */
    Plane& operator[] (int index);
    Plane const& operator[] (int index) const;
    Plane& operator() (int index);
    Plane const& operator() (int index) const;

    void print();

protected:
    std::vector<Plane> planeset;
};

PlaneSet::PlaneSet(const std::string& objfile) {
    std::ifstream ifs;
    ifs.open(objfile.data());
//    assert(ifs.is_open());

    std::string s;
    while (getline(ifs, s)){
        if(util::contain(s, "f")){
            std::vector<std::string> items = util::boostsplit(util::trim(s), " ");
            std::vector<int> pointsIndex;
            for (int i=1; i<items.size(); i++){
                pointsIndex.push_back(atoi(items[i].c_str())-1); // 面中取RAM索引
            }
            this->planeset.push_back(Plane(pointsIndex));
        }
    }
    ifs.close();
}



PlaneSet& PlaneSet::addPlane(std::vector<int> pointsIndex)
{
    this->planeset.push_back(Plane(pointsIndex));
}

PlaneSet& PlaneSet::addPlane(const Plane & plane)
{
    this->planeset.push_back(plane);
}

std::vector<Plane> PlaneSet::getPlaneSet()
{
    return this->planeset;
}

int PlaneSet::size() const {
    return this->planeset.size();
}

void PlaneSet::setPointIndex(int planeIndex, int pointIndex, int index)
{
    this->planeset[planeIndex].setPointIndex(pointIndex, index);
}

int PlaneSet::getPointIndex(int planeIndex, int pointIndex) const
{
    Plane plane = this->planeset[planeIndex];
    return plane.getPointIndex(pointIndex);
}

Plane& PlaneSet::operator[] (int index)
{
    return this->planeset[index];
}

Plane const& PlaneSet::operator[] (int index) const
{
    return this->planeset[index];
}

Plane& PlaneSet::operator() (int index)
{
    return this->planeset[index];
}

Plane const& PlaneSet::operator() (int index) const
{
    return this->planeset[index];
}

void PlaneSet::print()
{
    std::cout << "== plane set ==" << std::endl;

    for (int i=0; i<this->planeset.size(); i++){
        std::cout << "Plane[" << i << "]: [";
        planeset[i].print();
    }
    std::cout << "]" << std::endl;
}


GEOMETRY_NAMESPACE_END




#endif //BMEA_PLANESET_H
