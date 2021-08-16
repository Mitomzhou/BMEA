//
// Created by mitom on 8/10/21.
//
#include "string_util.h"

UTIL_NAMESPACE_BEGIN

/**
 * split boost实现
 * @param input 输入string
 * @param reg 正则符号
 * @return
 */
std::vector<std::string> boostsplit(const std::string& input, const std::string& reg)
{
    std::vector <std::string> fields;
    boost::split( fields, input, boost::is_any_of(reg) );
    return fields;
}

/**
 * split 正则实现
 * @param input 输入string
 * @param reg 正则符号
 * @return
 */
std::vector<std::string> regexsplit(const std::string& input, const std::string& reg)
{
    std::regex re(reg);

    std::sregex_token_iterator p(input.begin(), input.end(), re, -1);
    std::sregex_token_iterator end;
    std::vector<std::string> vec;
    while (p != end)
        vec.emplace_back(*p++);

    return vec;
}

/**
 * 是否包含子串
 * @param src
 * @param dst
 * @return
 */
bool contain(const std::string& src, const std::string& dst)
{
    std::string::size_type idx;
    idx = src.find(dst);
    return !(idx == std::string::npos);
}

/**
 * string to double 默认保留三位小数
 * @param str
 * @return
 */
double stod(std::string str)
{
    double ret = std::atof(str.c_str());
    double decimal = ret - (int)ret;
    decimal *= 1000;
    if((decimal-(int)decimal) >= 0.5) decimal += 1;
    decimal = (int)decimal;
    decimal /= 1000;
    decimal += (int)ret;
    return decimal;
}

/* 去除string末尾的空格 */
std::string& trim(std::string &s)
{
    if (s.empty())  return s;
    s.erase(0,s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" \r\n\t") + 1);
    return s;
}


UTIL_NAMESPACE_END