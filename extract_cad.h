//
// Created by mitom on 8/13/21.
//

#ifndef BMEA_EXTRACT_CAD_H
#define BMEA_EXTRACT_CAD_H

#include <iostream>
#include <string>
#include <vector>

#include<opencv2/opencv.hpp>

#include "./geometry/defines.h"
#include "./geometry/Point.h"
#include "./geometry/Plane.h"
#include "./geometry/PointCloud.h"
#include "./geometry/PlaneSet.h"
#include "./geometry/Contour.h"
#include "./util/math.h"

#include "./geometry/defines.h"

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

/* 计算偏转角 */
double getBuildingRotation(geometry::Contour& floorContour, geometry::PointCloud& pc);

/* link转换成多边形 */
std::vector<std::vector<int>> linkConvertPolygen(geometry::Contour& contour, int findValue);

/* 计算点是否在线上 */
bool pointOnLine(geometry::Point point1, geometry::Point point2, geometry::Point referPoint, double thredDistance); // thredDistance=0.3

/* 判断点是否在多边形内，计算遮挡关系 */
bool polygenContainsPoint(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc);

/* 计算内点 */
std::vector<int> calcInnerPoints(std::vector<std::vector<std::vector<int>>>& multlayerPolygensContour,
                                 const geometry::PointCloud& pc, std::vector<std::pair<int,int>>& verticalMergeTuples);


/**
 * 提取cad的主体函数
 * @param height_v
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

        //if(非俯视图) // TODO
        if(contour.getLinks().size() != 0){
            std::vector<std::vector<int>> polygensContour = linkConvertPolygen(contour, contour.getLinks()[0].first);
            multlayerPolygensContour.push_back(polygensContour);
        }

        //draw_contour(contour, pc);
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

    // std::cout << "所有点垂直替换关系" << verticalMergeTuples.size() << std::endl;
    // std::cout << std::endl;
    geometry::Contour result = replaceLinkPoint(verticalMergeTuples, allContour);

    std::cout << std::endl << "最终的contour:" << result.size() << std::endl;
    for (auto t : result.getLinks()){
        std::cout << "(" << t.first+1 << "," << t.second+1 << ")" << " ";
    }
    std::cout << std::endl;
    // draw_contour(result, pc);


    std::vector<int> innerPointsIndex = calcInnerPoints(multlayerPolygensContour, pc, verticalMergeTuples);
    for(auto p : innerPointsIndex)  std::cout << p << ",";
    std::cout << std::endl;


    return result;
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
    // 合并点前关系
    /**
    std::cout << "合并点前关系： " << mergeRowList_tuple.size() << std::endl;
    for (auto t : mergeRowList_tuple){
        std::cout << "(" << t[0] << "," << t[1] << ")" << " ";
    }
    std::cout << std::endl;
    std::cout << "合并点后关系： " << mergeResult.size() << std::endl;
    for (auto t : mergeResult){
        std::cout << "(" << t.first << "," << t.second << ")" << " ";
    }
    std::cout << std::endl;
     */
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
    // contour.print();
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
    int slace = 5;
    const char* filename = "/home/mitom/bgb.png";
    cv::Mat mat = cv::imread(filename);
    cv::Scalar color = cv::Scalar(0, 0, 255);
    for(auto link : contour.getLinks()){
        double x1 = pc.getPointSet()[link.first].getX()-100;
        double y1 = pc.getPointSet()[link.first].getY();
        double x2 = pc.getPointSet()[link.second].getX()-100;
        double y2 = pc.getPointSet()[link.second].getY();
        if(true){
            x1 *= slace; y1 *= slace; x2 *= slace; y2 *= slace;
        }

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
            x1 += 0; y1 += 500; x2 += 0; y2 += 500;
        }

        cv::Point2d p1 = cv::Point2d(x1, y1);
        cv::Point2d p2 = cv::Point2d(x2, y2) ;
        cv::line(mat ,p1,p2,color,1,cv::LINE_8);
    }
    cv::imshow("mat",mat);
    cv::waitKey();
}

/* 在floor层中查找最长的轮廓线来定建筑方向 */
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
 * 将link转为首尾相接polygen
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

bool polygenContainsPoint(const geometry::Point& referPoint, const std::vector<int>& polygen, const geometry::PointCloud& pc)
{
    for(int i=0; i<polygen.size()-1; i++){
        if(pointOnLine(pc.getPointSet()[i], pc.getPointSet()[i+1], referPoint, 0.3)){
            return false;
        }
    }
    int crossing = 0;
    for(int i=0; i<polygen.size()-1; i++){
        double slope = 0;
        if (pc.getPointSet()[i+1].getX() - pc.getPointSet()[i].getX() != 0){
            slope = (pc.getPointSet()[i+1].getY() - pc.getPointSet()[i].getY()) / (pc.getPointSet()[i+1].getX() - pc.getPointSet()[i].getX());
        }
        bool condition1 = (pc.getPointSet()[i].getX() < referPoint.getX()) && (referPoint.getX() < pc.getPointSet()[i+1].getX());
        bool condition2 = (pc.getPointSet()[i+1].getX() < referPoint.getX()) && (referPoint.getX() < pc.getPointSet()[i].getX());
        bool above = (referPoint.getY() < slope * (referPoint.getX() - pc.getPointSet()[i].getX() + pc.getPointSet()[i].getX()) );
        if((condition1 || condition2) && above){
            crossing ++;
        }
    }
    return (crossing % 2 != 0);
}

std::vector<int> calcInnerPoints(std::vector<std::vector<std::vector<int>>>& multlayerPolygensContour, const geometry::PointCloud& pc, std::vector<std::pair<int,int>>& verticalMergeTuples)
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
                    int referPointIndex = multlayerPolygensContour[i][j][k];
                    geometry::Point referPoint = pc.getPointSet()[referPointIndex];
                    for (int c=0; c<toplayerPolygens.size(); c++){
                        std::vector<int> polygen;
                        polygen = (toplayerPolygens[i]);
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
                innerPointsIndex[i] == tuple.first;
            }
        }
    }
    return innerPointsIndex;
}


#endif //BMEA_EXTRACT_CAD_H
