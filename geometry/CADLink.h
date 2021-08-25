//
// Created by mitom on 8/25/21.
//

#ifndef BMEA_CADLINK_H
#define BMEA_CADLINK_H

#include <iostream>
#include <vector>
#include "defines.h"

GEOMETRY_NAMESPACE_BEGIN

class CADLink {
public:
    CADLink(void) = default;
    CADLink(double x1v, double y1v, double x2v, double y2v) : x1(x1v), y1(y1v), x2(x2v), y2(y2v) {}
    CADLink(CADLink const& other); // copy
    double getX1() const { return this->x1; }
    double getY1() const { return this->y1; }
    double getX2() const { return this->x2; }
    double getY2() const { return this->y2; }
    void setX1(const double& x1v);
    void setY1(const double& y1v);
    void setX2(const double& x2v);
    void setY2(const double& y2v);
    void print();
protected:
    double x1;
    double y1;
    double x2;
    double y2;
};

inline
CADLink::CADLink(CADLink const& other) {
this->x1 = other.x1;
this->y1 = other.y1;
this->x2 = other.x2;
this->y2 = other.y2;
}

inline
void CADLink::setX1(const double& x1v)
{
    this->x1 = x1v;
}

inline
void CADLink::setY1(const double& y1v)
{
    this->y1 = y1v;
}

inline
void CADLink::setX2(const double& x2v)
{
    this->x2 = x2v;
}

inline
void CADLink::setY2(const double& y2v)
{
    this->y2 = y2v;
}

inline
void CADLink::print()
{
    std::cout << "CADLink: (" << x1 << "," << y1
    << ") (" << x2 << "," << y2 << ")" ;
}


GEOMETRY_NAMESPACE_END

#endif //BMEA_CADLINK_H
