//
// Created by mitom on 8/10/21.
//

#ifndef BMEA_STRING_UTIL_H
#define BMEA_STRING_UTIL_H

#include <string>
#include "defines.h"

UTIL_NAMESPACE_BEGIN

/* split */
std::vector<std::string> boostsplit(const std::string& input, const std::string& reg);
std::vector<std::string> regexsplit(const std::string& input, const std::string& reg);

/* 包含 */
bool contain(const std::string& src, const std::string& dst);

/* string to double */
double stod(std::string str);

/* 去除string末尾的空格 */
std::string& trim(std::string &s);

std::string addtoFilename(std::string filename, const std::string& tag);

/* 替换文件后缀名 */
std::string replaceSuffix(const std::string& filename,  const std::string& suffix);

UTIL_NAMESPACE_END

#endif //BMEA_STRING_UTIL_H
