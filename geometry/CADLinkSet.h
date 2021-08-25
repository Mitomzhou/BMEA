//
// Created by mitom on 8/25/21.
//

#ifndef BMEA_CADLINKSET_H
#define BMEA_CADLINKSET_H

#include <iostream>
#include <vector>
#include "defines.h"
#include "CADLink.h"

GEOMETRY_NAMESPACE_BEGIN

class CADLinkSet {
public:
    CADLinkSet(void) = default;
    CADLinkSet(std::vector<CADLink> cadlinks);

    CADLinkSet &addLink(double x1, double y1, double x2, double y2);
    CADLinkSet &addLink(const CADLink &cadlink);
    std::vector<CADLink> getCADLinkSet() const;

    /** 对象操作 */
    CADLink &operator[](int index);
    CADLink const &operator[](int index) const;
    CADLink &operator()(int index);
    CADLink const &operator()(int index) const;

    int size() const;
    void print();

protected:
    std::vector<CADLink> cadlinkset;
};

inline
CADLinkSet::CADLinkSet(std::vector<CADLink> cadlinks)
{
    this->cadlinkset = cadlinks;
}

inline
CADLinkSet& CADLinkSet::addLink(double x1, double y1, double x2, double y2)
{
    this->cadlinkset.push_back(CADLink(x1, y1, x2, y2));
    return *this;
}

inline
CADLinkSet& CADLinkSet::addLink(const CADLink & cadlink)
{
    this->cadlinkset.push_back(cadlink);
    return *this;
}

inline
std::vector<CADLink> CADLinkSet::getCADLinkSet() const
{
    return this->cadlinkset;
}

inline
CADLink& CADLinkSet::operator[](int index)
{
    return this->cadlinkset[index];
}

inline
CADLink const& CADLinkSet::operator[](int index) const
{
    return this->cadlinkset[index];
}

inline
CADLink& CADLinkSet::operator()(int index)
{
    return this->cadlinkset[index];
}

inline
CADLink const& CADLinkSet::operator()(int index) const
{
    return this->cadlinkset[index];
}

inline
int CADLinkSet::size() const
{
    return this->cadlinkset.size();
}

inline
void CADLinkSet::print()
{
    std::cout << "== cad link set ==" << std::endl;

    for (int i=0; i<this->cadlinkset.size(); i++){
        std::cout << "CADLink[" << i << "]: [";
        cadlinkset[i].print();
    }
    std::cout << "]" << std::endl;
}

GEOMETRY_NAMESPACE_END

#endif //BMEA_CADLINKSET_H
