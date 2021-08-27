//
// Created by mitom on 8/11/21.
//

#ifndef BMEA_MATHOP_H
#define BMEA_MATHOP_H

#include "defines.h"

UTIL_NAMESPACE_BEGIN

double distance(double x1, double x2);

int argmax(std::vector<double>& data);

double setPrecision(double num, int digitnum);

double calcDistance(double x1, double y1, double x2, double y2);

double calcRadian(double x1, double y1, double x2, double y2);

std::vector<double> linefun(double x1, double y1, double x2, double y2);


/* vector包含vector */
bool contain_v(std::vector<int> src, std::vector<int> det);

/* vector包含item元素 */
bool contain_i(std::vector<int> src, int item);

/* 去重 */
//template <typename T>
void doUnique(std::vector<int>& src);

void doUnique_d(std::vector<double>& src);

/* 比较函数，大到小顺序, std::sort(v.begin(), v,end(), cmp_sort) */
bool cmp_sort(double x1, double x2);

std::pair<int, int> pair_sort(std::pair<int, int> link);

/* link 去重 */
std::vector<std::pair<int, int>> doUnique_pairv(std::vector<std::pair<int, int>> links);

/* 查找元素第一次出现的下标 */
int find_e(const std::vector<int>& vec, const int element);

/* 移除vector下标index元素 */
void remove(std::vector<int>& vec, int index);


UTIL_NAMESPACE_END

#endif //BMEA_MATHOP_H
