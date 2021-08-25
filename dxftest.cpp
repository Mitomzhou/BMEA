//
// Created by mitom on 8/25/21.
//

#include "dxftest.h"

int drawDXF(const char* outdxf, const geometry::CADLinkSet& linkset)
{
    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out(outdxf, DL_Codes::AC1015);

    // section header:
    dxf.writeHeader(*dw);
    dw->sectionEnd();

    // section tables:
    dw->sectionTables();

    // VPORT:
    dxf.writeVPort(*dw);

    // LTYPE:
    dw->tableLinetypes(1);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
    dw->tableEnd();

    // LAYER:
    dw->tableLayers(1); // layer数
    dxf.writeLayer( *dw, DL_LayerData("0", 0),   // DL_LayerData有三种: 0 mainlayer anotherlayer
                    DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS"));  // 定义line颜色 宽度(单位1/100mm) style
    dw->tableEnd();

    // STYLE:
    dw->tableStyle(1);
    DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
    style.bold = false;
    style.italic = false;
    dxf.writeStyle(*dw, style);
    dw->tableEnd();

    // VIEW:
    dxf.writeView(*dw);

    // UCS:
    dxf.writeUcs(*dw);

    // APPID:
    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    // DIMSTYLE:
    dxf.writeDimStyle(*dw, 2.5, 0.625, 0.625, 0.625, 2.5);  // 定义了外观尺度

    // BLOCK_RECORD:
    dxf.writeBlockRecord(*dw);
    dw->tableEnd();

    dw->sectionEnd();

    // BLOCK:
    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    // ENTITIES:
    dw->sectionEntities();

    DL_Attributes attributes("0", 256, -1, -1, "BYLAYER"); // DL_LayerData三种其中一种:0  layer颜色：256  layer width:-1 line style:BYLAYER

    // LINE:
//    DL_LineData lineData(10, 5, 0, 30, 5, 0); // 起始点：10, 5  结束点: 30, 5
//    dxf.writeLine(*dw, lineData, attributes);
//    DL_LineData lineData2(20, -30, 0, 20, 50, 0); // 起始点：10, 5  结束点: 30, 5
//    dxf.writeLine(*dw, lineData2, attributes);

    // 画线
    std::vector<geometry::CADLink> cadlinks = linkset.getCADLinkSet();
    for(int i=0; i<cadlinks.size(); i++){
        double x1 = cadlinks[i].getX1();
        double y1 = cadlinks[i].getY1();
        double x2 = cadlinks[i].getX2();
        double y2 = cadlinks[i].getY2();
        dxf.writeLine(*dw, DL_LineData(x1,y1,0,x2,y2,0), attributes);
    }

    //    // DIMENSION: 测量标尺等
    //    DL_DimensionData dimData(10.0,50.0,0.0,0,0,0.0,0x1,8,2,1.0,"","Standard",0.0,1.0,1.0);
    //    DL_DimAlignedData dimAlignedData(10.0,5.0,0.0,30.0,5.0,0.0);
    //
    //    dxf.writeDimAligned(*dw, dimData, dimAlignedData, attributes);

    // end section ENTITIES:
    dw->sectionEnd();
    dxf.writeObjects(*dw, "MY_OBJECTS");
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;
    std::cout << "-- DXF file write done! --" << std::endl;

    return 0;
}

geometry::CADLinkSet convertCoodsCADLink(geometry::Contour& contour, const geometry::PointCloud& pc)
{
    std::vector<geometry::CADLink> cadlinkset;
    std::vector<std::pair<int, int>> contourLinks = contour.getLinks();
    for(auto link : contourLinks){
        double x1 = pc[link.first].getX();
        double y1 = pc[link.first].getY();
        double x2 = pc[link.second].getX();
        double y2 = pc[link.second].getY();
        geometry::CADLink cadlink(x1, y1, x2, y2);
        cadlinkset.push_back(cadlink);
    }
    return geometry::CADLinkSet(cadlinkset);
}