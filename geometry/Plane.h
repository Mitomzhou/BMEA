//
// Created by mitom on 8/12/21.
//

#ifndef BMEA_PLANE_H
#define BMEA_PLANE_H

#include <iostream>
#include <vector>
#include "defines.h"

GEOMETRY_NAMESPACE_BEGIN

class Plane{
public:
    Plane(void) = default;
    Plane(std::vector<int> indexs);
    Plane(Plane const& other); // copy

    std::vector<int> getPointsIndex();
    void setPointIndex(int pointIndex, int index);
    int getPointIndex(int pointIndex);
    void print();

protected:
    std::vector<int> pointsIndex;
};

Plane::Plane(std::vector<int> indexs)
{
    this->pointsIndex = indexs;
}

Plane::Plane(Plane const& other)
{
    this->pointsIndex = other.pointsIndex;
}

std::vector<int> Plane::getPointsIndex()
{
    return this->pointsIndex;
}

void Plane::setPointIndex(int pointIndex, int index)
{
    this->pointsIndex[pointIndex] = index;
}

int Plane::getPointIndex(int pointIndex)
{
    return this->pointsIndex[pointIndex];
}

void Plane::print()
{
    std::cout << "Plane [" ;
    for (auto index : pointsIndex){
        std::cout << index << "," ;
    }
    std::cout << "]" << std::endl;
}




GEOMETRY_NAMESPACE_END

#endif //BMEA_PLANE_H
