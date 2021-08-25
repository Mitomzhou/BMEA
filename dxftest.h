//
// Created by mitom on 8/25/21.
//

#ifndef BMEA_DXFTEST_H
#define BMEA_DXFTEST_H

#include <iostream>
#include <string>
#include "./dxf/dl_dxf.h"
#include "./geometry/CADLink.h"
#include "./geometry/CADLinkSet.h"
#include "./geometry/Contour.h"
#include "./geometry/PointCloud.h"

int drawDXF(const char* outdxf, const geometry::CADLinkSet& linkset);

geometry::CADLinkSet convertCoodsCADLink(geometry::Contour& contour, const geometry::PointCloud& pc);


#endif //BMEA_DXFTEST_H
