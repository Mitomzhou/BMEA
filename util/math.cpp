//
// Created by mitom on 8/11/21.
//
#include <math.h>
#include "math.h"
#include "defines.h"


UTIL_NAMESPACE_BEGIN

double distance(double x1, double x2)
{
    return sqrt((x1-x2) * (x1-x2));
}

int argmax(std::vector<double>& data)
{
    double maxvalue = -1;
    int maxindex = 0;
    for (int i=0; i<data.size(); i++){
        if(data[i] > maxvalue){
            maxvalue = data[i];
            maxindex = i;
        }
    }
    return maxindex;
}

bool contain_v(std::vector<int> src, std::vector<int> det)
{
    if(src.size()==0 || det.size()==0)  return false;
    bool result = true;
    for (auto d : det){
        result = (result && std::count(src.begin(), src.end(), d));
    }
    return result;
}

bool contain_i(std::vector<int> src, int item)
{
    return std::count(src.begin(), src.end(), item);
}

double setPrecision(double ret, int digitnum)
{
    double decimal = ret - (int)ret;
    decimal *= pow(10, digitnum);
    if((decimal-(int)decimal) >= 0.5) decimal += 1;
    decimal = (int)decimal;
    decimal /= pow(10, digitnum);
    decimal += (int)ret;
    return decimal;
}

double calcDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void doUnique(std::vector<int>& vec)
{
    sort(vec.begin(),vec.end());
    auto it = unique(vec.begin(), vec.end());
    vec.erase(it, vec.end());
}

void doUnique_d(std::vector<double>& vec)
{
    sort(vec.begin(),vec.end());
    auto it = unique(vec.begin(), vec.end());
    vec.erase(it, vec.end());
}

bool cmp_sort(double x1, double x2)
{
    return x1 < x2;
}

std::pair<int, int> pair_sort(std::pair<int, int> link)
{
    int a = link.first;
    int b = link.second;
    if (a > b) {
        link.first = b;
        link.second = a;
    }
    return link;
}

std::vector<std::pair<int, int>> doUnique_pairv(std::vector<std::pair<int, int>> links)
{
    std::vector<std::pair<int, int>> tmplinks;
    std::vector<std::pair<int, int>> resultContour;
    for(auto link : links){  // 序号小的放前面, 便于去重
        int left = link.first < link.second ? link.first : link.second;
        int right = link.first > link.second ? link.first : link.second;
        tmplinks.push_back(std::make_pair(left, right));
    }
    std::list<std::pair<int, int>> listPair;
    std::list<std::pair<int, int>>::iterator it;

    for(auto link : tmplinks){
        listPair.push_back(link);
    }

    listPair.sort();
    listPair.unique();

    for(it=listPair.begin(); it!=listPair.end(); ++it){
        resultContour.push_back(std::make_pair((*it).first, (*it).second));
    }
    return resultContour;
}

UTIL_NAMESPACE_END