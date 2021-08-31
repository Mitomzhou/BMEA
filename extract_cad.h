//
// Created by mitom on 8/13/21.
//

#ifndef BMEA_EXTRACT_CAD_H
#define BMEA_EXTRACT_CAD_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include<opencv2/opencv.hpp>

#include "./geometry/defines.h"
#include "./geometry/Point.h"
#include "./geometry/Plane.h"
#include "./geometry/PointCloud.h"
#include "./geometry/PlaneSet.h"
#include "./geometry/Contour.h"
#include "./util/mathop.h"

#include "./geometry/defines.h"

/* 提取算法主体 */
geometry::Contour extract(std::vector<double>& height_v, geometry::PointCloud& pc, geometry::PlaneSet& ps);

/* 合并点函数, 对于同层 */
std::vector<std::pair<int,int>> mergePoint(std::vector<int>& pointsIndex, geometry::PointCloud& pc, double threshold);

/* 更新面中点（合并关系） */
geometry::PlaneSet replaceMergePoints(std::vector<std::pair<int,int>>& mergeTuples, std::vector<int>& layerPlanesIndex, geometry::PlaneSet& ps);

/* 获取外轮廓 */
geometry::Contour connectContour(geometry::PlaneSet layerps);

/* 替换contour中link点, 和同层面中替换聚类中点有所不同，同层面涉及面的构成问题，而link中不存在，直接去重就可以了 */
geometry::Contour replaceLinkPoint(std::vector<std::pair<int,int>>& verticalMergeTuples, geometry::Contour& allContour);

/* 两个面中删除公共边 */
geometry::Contour removeCommonEdge(geometry::Contour& contour);

/* 是否有包含link */
bool containLink(geometry::Contour& contour, std::pair<int, int>& dst);

void draw_contour(geometry::Contour& contour, geometry::PointCloud& pc);
void draw_contour_2l(geometry::Contour& contour, geometry::PointCloud& pc);

/* 计算偏转角，这部分放python代码里面了 */
double getBuildingRotation(geometry::Contour& floorContour, geometry::PointCloud& pc);

/* link转换成多边形 */
std::vector<std::vector<int>> linkConvertPolygen(geometry::Contour& contour, int findValue);

/* 点是否在多边形的线上 */
bool pointOnPolygenLine(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc);

/* 计算点是否在线上 */
bool pointOnLine(geometry::Point point1, geometry::Point point2, geometry::Point referPoint, double thredDistance); // thredDistance=0.3

/* 判断点是否在多边形内，计算遮挡关系 */
bool polygenContainsPoint(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc);

/* 计算内点 */
std::vector<int> calcInnerPoints(std::vector<std::vector<std::vector<int>>>& multlayerPolygensContour,
                                 const geometry::PointCloud& pc, std::vector<std::pair<int,int>>& verticalMergeTuples, std::vector<std::vector<int>>& allOnlinePolygens);

/* 把多点共线拉直 */
geometry::Contour refineLine(geometry::Contour& contour, const geometry::PointCloud& pc);

/* 计算两个向量夹角 */
double calcLineAngle(int commonPoint, int p1, int p2, const geometry::PointCloud& pc);

/**
 * 提取cad的主体函数
 * @param height_v 高度是从高到低，便于计算遮挡关系
 * @param pc
 * @param ps
 */
geometry::Contour extract(std::vector<double>& height_v, geometry::PointCloud& pc, geometry::PlaneSet& ps)
{
    // 所有层的contour
    geometry::Contour allContour;
    // 所有层的点(同层平面点是替换过的)
    std::vector<int> allPoints;
    // 非俯视图的首尾相接的polygen，用于立面结构遮挡关系判断: 三维结构，层-》单层多边形几何-》单个多边形点
    std::vector<std::vector<std::vector<int>>> multlayerPolygensContour;

    std::vector<std::vector<int>> allLayerPointsIndex;
    std::vector<double> zcol = pc.getZCol();
    for (int i=0; i<height_v.size(); i++) {
        // 当层点
        std::vector<int> layerPointsIndex;
        for(int j=0; j<zcol.size(); j++){
            if(height_v[i] == zcol[j])
                layerPointsIndex.push_back(j); // RAM索引
        }
        // 当层面
        std::vector<int> layerPlanesIndex;
        for (int m=0; m<ps.getPlaneSet().size(); m++){
            std::vector<geometry::Plane> planes = ps.getPlaneSet();
            if(util::contain_v(layerPointsIndex, planes[m].getPointsIndex())){
                layerPlanesIndex.push_back(m);
            }
        }
        // 面中所有的贡献点去重
        // 贡献该层面的点
        std::vector<int> contriLayerPoints;
        for (auto plane : layerPlanesIndex){
            for (auto point : ps.getPlaneSet()[plane].getPointsIndex()){
                contriLayerPoints.push_back(point);
            }
        }
        util::doUnique(contriLayerPoints);
        // std::cout << "当层面贡献点数： " << contriLayerPoints.size() << std::endl;
        // for (auto point : contriLayerPoints)  std::cout << point << " ";
        // std::cout << std::endl;

        std::vector<std::pair<int,int>> mergeTuples =  mergePoint(contriLayerPoints, pc, 0.1);

        // 单层面聚类点并替换后的面
        geometry::PlaneSet layerps = replaceMergePoints(mergeTuples, layerPlanesIndex, ps);

        // 得到单层轮廓线
        geometry::Contour contour = connectContour(layerps);
        // draw_contour(contour, pc);

        //if(非俯视图)
        if(contour.getLinks().size() != 0){
            std::vector<std::vector<int>> polygensContour = linkConvertPolygen(contour, contour.getLinks()[0].first);
            multlayerPolygensContour.push_back(polygensContour);
        }


        for (auto link : contour.getLinks()){
            allContour.addLink(link);
        }
        for (auto plane : layerps.getPlaneSet()){
            for(auto point : plane.getPointsIndex()){
                allPoints.push_back(point);
            }
        }
    }

    // 得到所有层的点替换关系
    std::vector<std::pair<int,int>> verticalMergeTuples =  util::doUnique_pairv(mergePoint(allPoints, pc, 1));

    // 全部在线上多边形
    std::vector<std::vector<int>> allOnlinePolygens;

    // 计算内点，同时存储所有在线上的多边形到allOnlinePolygens
    std::vector<int> innerPointsIndex = calcInnerPoints(multlayerPolygensContour, pc, verticalMergeTuples, allOnlinePolygens);

     // 全部在线上多边形组装到Contour
    geometry::Contour allOnlinePolygensContour;
    geometry::Contour allContourExOnline;
    for (auto polygen : allOnlinePolygens){
        for (int i=0; i<polygen.size()-1; i++){
            allOnlinePolygensContour.addLink(std::make_pair(polygen[i], polygen[i+1]));
        }
    }

    // 对所有contour去除 多边形（全部在线上的）的link关系
    for(auto link : allContour.getLinks()){
        if (!allOnlinePolygensContour.contain(link)){
            allContourExOnline.addLink(link);
        }
    }

    // 替换点（换马甲）
    geometry::Contour replaceResultContour = replaceLinkPoint(verticalMergeTuples, allContourExOnline);

    // 内点在replaceResultContour中的link中就去除
    geometry::Contour cadcontour;
    for (auto link : replaceResultContour.getLinks()){
        bool exist = false;
        for (auto point : innerPointsIndex){
            exist = (exist || (link.first == point || link.second == point));
        }
        if (!exist){
            cadcontour.addLink(link);
        }
    }
    // 这里做3次refine，一次不能把所有点全部拉直
    geometry::Contour firstRefineContour = refineLine(cadcontour, pc);
    geometry::Contour secondRefineContour = refineLine(firstRefineContour, pc);
    geometry::Contour thirdRefineContour = refineLine(secondRefineContour, pc);
    // geometry::Contour forthRefineContour = refineLine(thirdRefineContour, pc);
    return thirdRefineContour;
}


/**
 * 合并点函数, 对于同层
 * @param pointsIndex
 * @param ps
 * @param threshold  对于同层threshold=0.1 (m),不同层点合并hreshold=1.0 (m)
 * @return 替换tuple
 */
std::vector<std::pair<int,int>> mergePoint(std::vector<int>& pointsIndex, geometry::PointCloud& pc, double threshold)
{
    std::vector<std::vector<int>> mergeRowList;
    for(int i=0; i<pointsIndex.size(); i++){
        for (int j=0; j<pointsIndex.size(); j++){
            double distance = util::calcDistance(pc[pointsIndex[i]].getX(), pc[pointsIndex[i]].getY(), pc[pointsIndex[j]].getX(), pc[pointsIndex[j]].getY());
            std::vector<int> mergeRow;
            if(distance < threshold) {
                mergeRow.push_back(pointsIndex[j]);
            }
            if(mergeRow.size() > 0){
                mergeRow.insert(mergeRow.begin(), pointsIndex[i]);
                mergeRowList.push_back(mergeRow);
            }
        }
    }
    // 把序号小的放前面, 并且间接去重了
    std::vector<std::vector<int>> mergeRowList_tuple;
    for (auto row : mergeRowList){
        if(row[0] < row[1]){
            mergeRowList_tuple.push_back(row);
        }
    }
    // mergeRowList_tuple: (1,3)(1,5)(1,7) (3,5)(3,7) (5,7)
    // tuple 第1列
    std::vector<int> col1;
    // tuple 第2列
    std::vector<int> col2;
    for(auto row : mergeRowList_tuple){
        col1.push_back(row[0]);
        col2.push_back(row[1]);
    }
    // tuple中两个元素是否在第2列中，如 （5,7）如果在(3,5,7,5,7,7)中,该tuple就舍弃
    std::vector<std::pair<int,int>> mergeResult; // 结果tuple替换关系
    for(auto t : mergeRowList_tuple){
        if(util::contain_v(col2, t)) {
            continue;
        }else{
            std::pair<int,int> tuple(t[0],t[1]);
            mergeResult.push_back(tuple);
        }
    }
    return mergeResult;
}

/**
 * 替换面中点index
 * @param mergeTuples 替换
 * @param layerPlanesIndex
 * @param ps
 * @return
 */
geometry::PlaneSet replaceMergePoints(std::vector<std::pair<int,int>>& mergeTuples, std::vector<int>& layerPlanesIndex, geometry::PlaneSet& ps)
{
    geometry::PlaneSet srcPs;
    for (auto faceIndex : layerPlanesIndex){
        srcPs.addPlane(ps[faceIndex]);
    }
    for (auto tuple : mergeTuples){
        for (int p=0; p<srcPs.getPlaneSet().size(); p++){
            for(int i=0; i<srcPs.getPlaneSet()[p].getPointsIndex().size(); i++){
                if(tuple.second == srcPs.getPlaneSet()[p].getPointsIndex()[i]){
                    srcPs.setPointIndex(p, i, tuple.first); // 替换面中点index
                }
            }
        }
    }

    geometry::PlaneSet resultPs;
    // 给面中点去重，并清除面中线段问题（元素少于3个）
    for (auto plane : srcPs.getPlaneSet()){
        std::vector<int> pointsIndex;
        // 这里去重注意，不能把点顺序打乱
        for (auto point : plane.getPointsIndex()){
            if(!util::contain_i(pointsIndex, point))
                pointsIndex.push_back(point);
        }
        if(pointsIndex.size() > 3){
            resultPs.addPlane(pointsIndex);
        }
    }
    // resultPs.print();
    return resultPs;
}

/**
 * 单层给定多个面，得到Contour，其中包含去除面公共边
 * @param layerps
 * @return 单层轮廓线
 */
geometry::Contour connectContour(geometry::PlaneSet layerps)
{
    geometry::Contour contour;
    for(int i=0; i<layerps.getPlaneSet().size(); i++){
        int pointsize = layerps.getPlaneSet()[i].getPointsIndex().size();
        for(int j=0; j<pointsize; j++){
            std::pair<int, int> link;
            if(j+1 <= pointsize-1){
                link.first = layerps.getPointIndex(i,j);
                link.second = layerps.getPointIndex(i,j+1);
                contour.addLink(util::pair_sort(link));
            }
        }
        contour.addLink(util::pair_sort(std::make_pair(layerps.getPointIndex(i,0), layerps.getPointIndex(i, pointsize-1)))); // 第一个点和最后一个点的link
    }
    // 这里去重注意，对与两个面的公共边，要去除
    return removeCommonEdge(contour);
}

/**
 * 替换所有contour中的点, 并且去重
 * @param verticalMergeTuples
 * @param allContour
 * @return
 */
geometry::Contour replaceLinkPoint(std::vector<std::pair<int,int>>& verticalMergeTuples, geometry::Contour& allContour)
{
    geometry::Contour resultContour;
    // allContour.print();

    for(auto link : allContour.getLinks()){
        for (auto tuple : verticalMergeTuples){
            if(tuple.second == link.first){
                link.first = tuple.first;
            }
            if(tuple.second == link.second){
                link.second = tuple.first;
            }
        }
        resultContour.addLink(std::make_pair(link.first, link.second));
    }
    // contour去重
     return util::doUnique_pairv(resultContour.getLinks());
}

/**
 * 消除公共边
 * @param contour
 * @return
 */
geometry::Contour removeCommonEdge(geometry::Contour& contour)
{
    geometry::Contour tmpContour;
    geometry::Contour commonEdge;
    geometry::Contour resultContour;
    std::vector<std::pair<int, int>>::iterator it;
    for (auto link : contour.getLinks()){
        if(containLink(tmpContour, link)){
            commonEdge.addLink(link);
        }else{
            tmpContour.addLink(link);
        }
    }
    for(auto link : tmpContour.getLinks()){
        if(!containLink(commonEdge, link)){
            resultContour.addLink(link);
        }
    }
    return resultContour;
}

/**
 * 是否有包含link
 * @param contour
 * @param dst
 * @return
 */
bool containLink(geometry::Contour& contour, std::pair<int, int>& dst)
{
    for(auto link : contour.getLinks()){
        if(link.first == dst.first && link.second == dst.second ||
                link.second == dst.first && link.first == dst.second)
            return true;
    }
    return false;
}


void draw_contour(geometry::Contour& contour, geometry::PointCloud& pc)
{
    int slace = 3;
    const char* filename = "/home/mitom/bgb.png";
    cv::Mat mat = cv::imread(filename);
    cv::Scalar color = cv::Scalar(0, 0, 255);
    for(auto link : contour.getLinks()){
        double x1 = pc.getPointSet()[link.first].getX(); // result -100
        double y1 = pc.getPointSet()[link.first].getY();
        double x2 = pc.getPointSet()[link.second].getX();
        double y2 = pc.getPointSet()[link.second].getY();
        if(true){
            x1 *= slace; y1 *= slace; x2 *= slace; y2 *= slace;
        }
        if(false){
            x1 += 500; y1 += 0; x2 += 500; y2 += 0;
        }
        if(true){
            x1 += 0; y1 += 500; x2 += 0; y2 += 500;
        }
//        std::cout << x1 << ", "<< y1 << ", "<< x2 << ", "<< y2 << std::endl;
        cv::Point2d p1 = cv::Point2d(x1, y1);
        cv::Point2d p2 = cv::Point2d(x2, y2) ;
        cv::line(mat ,p1,p2,color,1,cv::LINE_8);
    }
    cv::imshow("mat",mat);
    cv::waitKey();
}

void draw_contour_2l(geometry::Contour& contour, geometry::PointCloud& pc)
{
    int slace = 2;
    const char* filename = "/home/mitom/bgb.png";
    cv::Mat mat = cv::imread(filename);
    cv::Scalar color = cv::Scalar(0, 0, 255);
    for(auto link : contour.getLinks()){
        double x1 = pc.getPointSet()[link.first].getX();
        double y1 = pc.getPointSet()[link.first].getY();
        double x2 = pc.getPointSet()[link.second].getX();
        double y2 = pc.getPointSet()[link.second].getY();
        if(true){
            x1 *= slace; y1 *= slace; x2 *= slace; y2 *= slace;
        }
        if(true){
            x1 += 400; y1 += 0; x2 += 400; y2 += 0;
        }
        cv::Point2d p1 = cv::Point2d(x1, y1);
        cv::Point2d p2 = cv::Point2d(x2, y2);

        cv::line(mat ,p1,p2,color,1,cv::LINE_8);
    }
    cv::imshow("mat",mat);
    cv::waitKey();
}

/**
 * 在floor层中查找最长的轮廓线来定建筑方向
 * @param floorContour
 * @param pc
 * @return
 */
double getBuildingRotation(geometry::Contour& floorContour, geometry::PointCloud& pc)
{
    // 获取最长线
    std::vector<double> linkLenth;
    for (auto link : floorContour.getLinks()) {
        double x1 = pc.getPointSet()[link.first].getX();
        double y1 = pc.getPointSet()[link.first].getY();
        double x2 = pc.getPointSet()[link.second].getX();
        double y2 = pc.getPointSet()[link.second].getY();
        linkLenth.push_back(util::calcDistance(x1, y1, x2, y2));
    }
    std::vector<double>::iterator biggest = std::max_element(std::begin(linkLenth), std::end(linkLenth));
    int longestIndex = std::distance(std::begin(linkLenth), biggest);

    // 计算最长线的偏转角
    double px1 = pc.getPointSet()[floorContour[longestIndex].first].getX();
    double py1 = pc.getPointSet()[floorContour[longestIndex].first].getY();
    double px2 = pc.getPointSet()[floorContour[longestIndex].second].getX();
    double py2 = pc.getPointSet()[floorContour[longestIndex].second].getY();

    return util::calcRadian(px1, py1, px2, py2);
}

/**
 * 将link转为首尾相接polygen (这里最好建一个Polygen类)
 * @param contour
 * @param findValue
 * @return
 */
std::vector<std::vector<int>> linkConvertPolygen(geometry::Contour& contour, int findValue)
{
    std::vector<int> first;
    std::vector<int> second;
    bool bfindValue = true;

    for (auto link : contour.getLinks()) {
        first.push_back(link.first);
        second.push_back(link.second);
    }
    std::vector<std::vector<int>> polygensContour; // 单层多个多边形contour
    std::vector<int> onePolyenLink;  // 一个多边形的连续link点，首尾相接

    while(first.size() != 0){
        int itemIndex = 0;
        int itemValue = 0;
        if (std::count(first.begin(), first.end(), findValue) != 0){
            itemIndex = util::find_e(first, findValue);
            itemValue = second[itemIndex];
            bfindValue = true;
            findValue = itemValue;
        }else if(std::count(second.begin(), second.end(), findValue) != 0){
            itemIndex = util::find_e(second, findValue);
            itemValue = first[itemIndex];
            bfindValue = true;
            findValue = itemValue;
        }else{
            bfindValue = false;
            findValue = first[0];
        }
        util::remove(first, itemIndex);
        util::remove(second, itemIndex);


        if (bfindValue) {
            onePolyenLink.push_back(findValue);
        }else{ // tail - top
            onePolyenLink.push_back(onePolyenLink[0]);
            polygensContour.push_back(onePolyenLink);
            onePolyenLink.clear();
            onePolyenLink.push_back(findValue);
        }
        if(first.size() == 0){
            onePolyenLink.push_back(onePolyenLink[0]);
            polygensContour.push_back(onePolyenLink);
        }
    }
    if(false){ // print
        std::cout << "pplygen num: " << polygensContour.size() << std::endl;
        for(auto contour : polygensContour){
            for(auto e : contour)  std::cout << e << ",";
            std::cout << std::endl;
        }
    }
    return polygensContour;
}

bool pointOnLine(geometry::Point point1, geometry::Point point2, geometry::Point referPoint, double thredDistance)
{
    std::vector<double> coeff = util::linefun(point1.getX(), point1.getY(), point2.getX(), point2.getY());
    if (point1.getX() <= referPoint.getX() && referPoint.getX() <= point2.getX() ||
        point2.getX() <= referPoint.getX() && referPoint.getX() <= point1.getX()){
        // 点到直线的距离公式
        double distance = abs(coeff[0] * referPoint.getX() + coeff[1] * referPoint.getY() + coeff[2])/sqrt(coeff[0] * coeff[0] + coeff[1] * coeff[1]);
        if (distance <= thredDistance){
            return true;
        }
    }else{
        double distance1 = util::calcDistance(referPoint.getX(), referPoint.getY(), point1.getX(), point1.getY());
        double distance2 = util::calcDistance(referPoint.getX(), referPoint.getY(), point2.getX(), point2.getY());
        if (distance1 <= thredDistance || distance2 <= thredDistance){
            return true;
        }
    }
    return false;
}

/**
 * 点是否在多边形线上, 阈值0.3m
 * @param referPoint
 * @param polygen
 * @param pc
 * @return
 */
bool pointOnPolygenLine(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc)
{
    for(int i=0; i<polygen.size()-1; i++){
        geometry::Point point1 = pc.getPointSet()[polygen[i]];
        geometry::Point point2 = pc.getPointSet()[polygen[i+1]];
        if(pointOnLine(point1, point2, referPoint, 0.3)){
            return true;
        }
    }
    return false;
}

/**
 * 判断点是否在多边形内，用来计算遮挡关系
 * Reference from: https://www.cnblogs.com/luxiaoxun/p/3722358.html
 * @param referPoint
 * @param polygen
 * @param pc
 * @return
 */
bool polygenContainsPoint(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc)
{
    // 点在线上
    for(int i=0; i<polygen.size()-1; i++){
        geometry::Point point1 = pc.getPointSet()[polygen[i]];
        geometry::Point point2 = pc.getPointSet()[polygen[i+1]];
        if(pointOnLine(point1, point2, referPoint, 0.3)){
            return false;
        }
    }
    // 点在外或在内
    int crossing = 0;
    for(int i=0; i<polygen.size()-1; i++){
        double slope = 0;
        geometry::Point point1 = pc.getPointSet()[polygen[i]];
        geometry::Point point2 = pc.getPointSet()[polygen[i+1]];
        if (point2.getX() - point1.getX() != 0){
            slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
        }
        bool condition1 = (point1.getX() <= referPoint.getX()) && (referPoint.getX() < point2.getX());
        bool condition2 = (point2.getX() <= referPoint.getX()) && (referPoint.getX() < point1.getX());
        bool above = (referPoint.getY() < slope * (referPoint.getX() - point1.getX()) + point1.getY() );
        if((condition1 || condition2) && above){
            crossing ++;
        }
    }
    return (crossing % 2 != 0);
}

/**
 * 计算内点，顺便计算一些多边形是否全部在上层contour上，如果全部在，则后续一些关系去除
 * @param multlayerPolygensContour 多层多个多边形
 * @param pc
 * @param verticalMergeTuples 垂直点替换关系
 * @param allOnlinePolygens 全部在上层的线上的多边形
 * @return
 */
std::vector<int> calcInnerPoints(std::vector<std::vector<std::vector<int>>>& multlayerPolygensContour, const geometry::PointCloud& pc,
                                 std::vector<std::pair<int,int>>& verticalMergeTuples, std::vector<std::vector<int>>& allOnlinePolygens)
{
    std::vector<int> innerPointsIndex;
    std::vector<std::vector<int>> toplayerPolygens;
    for(int l=0; l<multlayerPolygensContour.size(); l++){
        for (int i=l; i<multlayerPolygensContour.size(); i++){
            if(i == l){ // 顶层面
                toplayerPolygens = multlayerPolygensContour[l];
                continue;
            }
            for (int j=0; j<multlayerPolygensContour[i].size(); j++){ // 多个多边形
                for (int k=0; k<multlayerPolygensContour[i][j].size(); k++){ // 单个多边形

                    // 用来多边形是否在全部在线上，若多边形全部在线上，则该多边形的关系后续是要去除的
                    std::vector<bool> onlineBoolV;
                    bool online = true;
                    int referPointIndex = multlayerPolygensContour[i][j][k];
                    geometry::Point referPoint = pc.getPointSet()[referPointIndex];
                    for (int c=0; c<toplayerPolygens.size(); c++) {
                        std::vector<int> polygen;
                        polygen = (toplayerPolygens[c]);
                        onlineBoolV.push_back(pointOnPolygenLine(referPoint, polygen, pc));
                    }
                    for(auto b : onlineBoolV){
                        online = (online && b);
                    }
                    if(online){
                        if(!util::contain_vv(allOnlinePolygens, multlayerPolygensContour[i][j])){ // 同步去重了
                            allOnlinePolygens.push_back(multlayerPolygensContour[i][j]);
                        }
                        continue;
                    }

                    // 判断是否在多边形内和线上
                    for (int c=0; c<toplayerPolygens.size(); c++){
                        std::vector<int> polygen;
                        polygen = (toplayerPolygens[c]);
                        if(polygenContainsPoint(referPoint, polygen, pc))
                        {
                            innerPointsIndex.push_back(referPointIndex);
                        }
                    }
                }
            }
        }
    }
    util::doUnique(innerPointsIndex); // 内点去重

    // 内点被“所有层的点替换关系 verticalMergeTuples” 过滤后，返回内点,原因：不然会找不到情况（因为它换马甲了）
    for (int i=0; i<innerPointsIndex.size(); i++){
        for(auto tuple : verticalMergeTuples){
            if(innerPointsIndex[i] == tuple.second){
                innerPointsIndex[i] = tuple.first;
            }
        }
    }
    return innerPointsIndex;
}

/**
 * 拉直直线
 * @param contour
 * @param pc
 */
geometry::Contour refineLine(geometry::Contour& contour, const geometry::PointCloud& pc)
{
    std::vector<std::pair<int, int>> links = contour.getLinks();
    std::vector<int> pointsIndex;              // 所有点下标
    std::vector<int> twoConnectionPointsIndex; // 仅有2个连通域的点
    for (auto link : links){
        pointsIndex.push_back(link.first);
        pointsIndex.push_back(link.second);
    }
    std::vector<int> uniquePoints = pointsIndex;
    util::doUnique(uniquePoints);
    for(auto point : uniquePoints){
        int count = std::count(pointsIndex.begin(), pointsIndex.end(), point);
        if(count == 2){
            twoConnectionPointsIndex.push_back(point);
        }
    }

    // std::cout << "2个连通域-------------------" << std::endl;
    // for (auto p : twoConnectionPointsIndex){
    //    std::cout << p << ", ";
    // }
    // std::cout << std::endl;

    geometry::Contour mergeContour;  //合并contour
    std::vector<int> tmppoint;   //
    std::vector<int> hiddenpoint; // 隐藏点

    for(auto p : twoConnectionPointsIndex) {
        std::vector<int> mergelink;
        for(auto link : contour.getLinks()) {
            if (!util::contain_i(tmppoint, p)){
                if (p == link.first) {
                    mergelink.push_back(link.second);
                }else if(p == link.second){
                    mergelink.push_back(link.first);
                }
                if(mergelink.size() == 2){
                    double cos = calcLineAngle(p, mergelink[0], mergelink[1], pc);
                    if (cos >= 0.965925826289 && cos <= 1){ // cos 10度 = 0.984807753   cos 15 =0.965925826289
                        mergeContour.addLink(std::make_pair(mergelink[0], mergelink[1]));
                        hiddenpoint.push_back(p);
                        tmppoint.push_back(p);
                        tmppoint.push_back(mergelink[0]);
                        tmppoint.push_back(mergelink[1]);
                    }
                }
            }
        }
    }

    // std::cout << "合并contour" << std::endl;
    // mergeContour.print();
    // std::cout << "隐藏点" << std::endl;
    //for(auto p : hiddenpoint)  std::cout << p << ",";
    // std::cout << std::endl;

    // 去除所有包含中间公共点的link，添加合并后的link
    geometry::Contour refineContour;
    for (auto link : contour.getLinks()){
        if (!(util::contain_i(hiddenpoint, link.first) || util::contain_i(hiddenpoint, link.second))){
            refineContour.addLink(link);
        }
    }
    for (auto link : mergeContour.getLinks()){
        refineContour.addLink(link);
    }
    // std::cout << "优化后 refine contour:" << std::endl;
    // refineContour.print();
    return refineContour;
}

/**
 * 计算两个向量夹角
 * @param commonPoint
 * @param p1
 * @param p2
 * @param pc
 * @return cos
 */
double calcLineAngle(int commonPoint, int p1, int p2, const geometry::PointCloud& pc)
{
    double comX = pc.getPointSet()[commonPoint].getX();
    double comY = pc.getPointSet()[commonPoint].getY();
    double p1X = pc.getPointSet()[p1].getX();
    double p1Y = pc.getPointSet()[p1].getY();
    double p2X = pc.getPointSet()[p2].getX();
    double p2Y = pc.getPointSet()[p2].getY();
    // v-a v-b
    double ax = comX - p1X;
    double ay = comY - p1Y;
    double bx = p2X - comX;
    double by = p2Y - comY;
    return (ax * bx + ay * by) / (sqrt(ax * ax + ay * ay) * sqrt(bx * bx + by * by));
}

#endif //BMEA_EXTRACT_CAD_H
