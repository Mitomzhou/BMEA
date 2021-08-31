//
// Created by mitom on 8/16/21.
//

#ifndef BMEA_CONTOUR_H
#define BMEA_CONTOUR_H

#include <iostream>
#include <vector>
#include "defines.h"
#include "CADLink.h"
#include "CADLinkSet.h"

GEOMETRY_NAMESPACE_BEGIN

class Contour {
public:
    Contour(void) = default;
    Contour(std::vector<std::pair<int, int>> links);
    Contour(Contour const& contour); // copy

    Contour& addLink(std::pair<int, int> link);

    std::vector<std::pair<int, int>> getLinks();

    /* 包含link */
    bool contain(const std::pair<int, int>& tuple);

    int size() const;

    /* 设置planeset 中某面某个点index */
    //void setPointIndex(int planeIndex, int pointIndex, int index);

    /** 对象操作 */
    std::pair<int, int>& operator[] (int index);
    std::pair<int, int> const& operator[] (int index) const;
    std::pair<int, int>& operator() (int index);
    std::pair<int, int> const& operator() (int index) const;

    void print();

protected:
    std::vector<std::pair<int, int>> links;
};

inline
Contour::Contour(std::vector<std::pair<int, int>> links)
{
    this->links = links;
}

inline
Contour::Contour(Contour const& contour)
{
    this->links = contour.links;
}

inline
Contour& Contour::addLink(std::pair<int, int> link)
{
    this->links.push_back(link);
}

inline
std::vector<std::pair<int, int>> Contour::getLinks()
{
    return this->links;
}

inline
bool Contour::contain(const std::pair<int, int>& tuple)
{
    for (auto link : this->links){
        if((link.first == tuple.first && link.second == tuple.second) || (link.second == tuple.first && link.first == tuple.second)){
            return true;
        }
    }
    return false;
}

inline
int Contour::size() const
{
    return this->links.size();
}

inline
std::pair<int, int>& Contour::operator[] (int index)
{
    return this->links[index];
}

inline
std::pair<int, int> const& Contour::operator[] (int index) const
{
    return this->links[index];
}

inline
std::pair<int, int>& Contour::operator() (int index)
{
    return this->links[index];
}

inline
std::pair<int, int> const& Contour::operator() (int index) const
{
    return this->links[index];
}

inline
void Contour::print()
{
    std::cout << "== contour ==" << this->links.size() << std::endl;
    for (int i=0; i<this->links.size(); i++){
        std::cout << "[" << this->links[i].first << "," << this->links[i].second << "] ";
        if((i+1) % 10 == 0)  std::cout << std::endl;
    }
    std::cout << std::endl;
}

GEOMETRY_NAMESPACE_END

#endif //BMEA_CONTOUR_H
