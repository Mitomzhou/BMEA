//
// Created by mitom on 8/11/21.
//

#ifndef BMEA_MATH_H
#define BMEA_MATH_H

#include "defines.h"

UTIL_NAMESPACE_BEGIN

double distance(double x1, double x2);

int argmax(std::vector<double>& data);

double setPrecision(double num, int digitnum);

double calcDistance(double x1, double y1, double x2, double y2);

/* vector包含vector */
bool contain_v(std::vector<int> src, std::vector<int> det);

/* vector包含item元素 */
bool contain_i(std::vector<int> src, int item);

/* 去重 */
void doUnique(std::vector<int>& src);

/* 比较函数，大到小顺序, std::sort(v.begin(), v,end(), cmp_sort) */
bool cmp_sort(double x1, double x2);

std::pair<int, int> pair_sort(std::pair<int, int> link);

/* link 去重 */
std::vector<std::pair<int, int>> doUnique_pairv(std::vector<std::pair<int, int>> links);

UTIL_NAMESPACE_END

#endif //BMEA_MATH_H
