//
// Created by mitom on 8/13/21.
//

#ifndef BMEA_EXTRACT_CAD_H
#define BMEA_EXTRACT_CAD_H

#include <iostream>
#include <string>
#include <vector>

#include "./geometry/defines.h"
#include "./geometry/Point.h"
#include "./geometry/Plane.h"
#include "./geometry/PointCloud.h"
#include "./geometry/PlaneSet.h"
#include "./geometry/Contour.h"
#include "./util/math.h"

#include "./geometry/defines.h"

void extract(std::vector<double>& height_v, geometry::PointCloud& pc, geometry::PlaneSet& ps);

/* 合并点函数, 对于同层 */
std::vector<std::pair<int,int>> mergePoint(std::vector<int>& pointsIndex, geometry::PointCloud& pc, double threshold);

/* 更新面中点（合并关系） */
geometry::PlaneSet replaceMergePoints(std::vector<std::pair<int,int>>& mergeTuples, std::vector<int>& layerPlanesIndex, geometry::PlaneSet& ps);

/* 获取外轮廓 */
geometry::Contour connectContour(geometry::PlaneSet layerps);

/* 替换contour中link点, 和同层面中替换聚类中点有所不同，同层面涉及面的构成问题，而link中不存在，直接去重就可以了 */
geometry::Contour replaceLinkPoint(std::vector<std::pair<int,int>>& verticalMergeTuples, geometry::Contour& allContour);

/**
 * 提取cad的主体函数
 * @param height_v
 * @param pc
 * @param ps
 */
void extract(std::vector<double>& height_v, geometry::PointCloud& pc, geometry::PlaneSet& ps)
{
    // 所有层的contour
    geometry::Contour allContour;
    // 所有层的点(同层平面点是替换过的)
    std::vector<int> allPoints;

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

        if(false){
            std::cout << "-------" << height_v[i] << std::endl;
            for (auto point : layerPointsIndex)  std::cout << point << " ";
            std::cout << std::endl;
            std::cout << "面数： " << layerPlanesIndex.size() << std::endl;
            for (auto plane : layerPlanesIndex)  std::cout << plane << " ";
            std::cout << std::endl;
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
        std::cout << "当层面贡献点数： " << contriLayerPoints.size() << std::endl;
        for (auto point : contriLayerPoints)  std::cout << point << " ";
        std::cout << std::endl;

        std::vector<std::pair<int,int>> mergeTuples =  mergePoint(contriLayerPoints, pc, 0.1);

        // 单层面聚类点并替换后的面
        geometry::PlaneSet layerps = replaceMergePoints(mergeTuples, layerPlanesIndex, ps);

        // 得到单层轮廓线
        geometry::Contour contour = connectContour(layerps);

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

    std::cout << "所有点垂直替换关系" << verticalMergeTuples.size() << std::endl;
    for (auto t : verticalMergeTuples){
        std::cout << "(" << t.first << "," << t.second << ")" << " ";
    }

    // TODO 检查
    geometry::Contour result = replaceLinkPoint(verticalMergeTuples, allContour);

    std::cout << std::endl << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
    for (auto t : result.getLinks()){
        std::cout << "(" << t.first << "," << t.second << ")" << " ";
    }

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
        std::vector<int> pointsIndex = plane.getPointsIndex();
        util::doUnique(pointsIndex);
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
    contour.print();
    geometry::Contour resultContour(util::doUnique_pairv(contour.getLinks()));
    resultContour.print();
    return resultContour;
}

/**
 * 替换所有contour中的点
 * @param verticalMergeTuples
 * @param allContour
 * @return
 */
geometry::Contour replaceLinkPoint(std::vector<std::pair<int,int>>& verticalMergeTuples, geometry::Contour& allContour)
{
    for (int i=0; i<allContour.getLinks().size(); i++){
        for (int j=0; j<verticalMergeTuples.size(); j++){
            if(verticalMergeTuples[j].second == allContour.getLinks()[i].first){
                allContour.getLinks()[i].first = verticalMergeTuples[j].first;
            }
            if(verticalMergeTuples[j].second == allContour.getLinks()[i].second){
                allContour.getLinks()[i].second = verticalMergeTuples[j].first;
            }
        }
    }
    return allContour;
    // TODO 去重

}


#endif //BMEA_EXTRACT_CAD_H
