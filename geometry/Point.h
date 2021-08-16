//
// Created by mitom on 8/10/21.
//

#ifndef BMEA_POINT_H
#define BMEA_POINT_H

#include <iostream>

#include "defines.h"

GEOMETRY_NAMESPACE_BEGIN

class Point{
public:
    Point(void) = default;
    Point(double xv, double yv, double zv) : x(xv), y(yv), z(zv) {}
    Point(double xv, double yv) : x(xv), y(yv), z(0) {}  // z=0 二维点
    Point(Point const& other); // copy
    double getX() const { return this->x; }
    double getY() const { return this->y; }
    double getZ() const { return this->z; }
    void setX(const double& xv);
    void setY(const double& yv);
    void setZ(const double& zv);
    void print();

protected:
    double x;
    double y;
    double z;
};

Point::Point(Point const& other) {
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
}

void Point::setX(const double& xv)
{
    this->x = xv;
}

void Point::setY(const double& yv)
{
    this->y = yv;
}

void Point::setZ(const double& zv)
{
    this->z = zv;
}

void Point::print() {
    std::cout << "Point (" << x << ", " << y << ", " << z << ")" << std::endl;
}

GEOMETRY_NAMESPACE_END



#endif //BMEA_POINT_H
