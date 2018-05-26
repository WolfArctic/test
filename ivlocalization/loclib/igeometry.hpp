#include "igeometry.h"

template<typename PointType>
igeometry2<PointType>::igeometry2() {}

template<typename PointType>
s2DLineEquationf igeometry2<PointType>::lineFitting(PointsTypePtr inputCloud, float inlierThre) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(inlierThre);
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);
    if (0 == inliers->indices.size()) {
        s2DLineEquationf li;
        li.isValid = false;
        return li;
    }
    s2DLineEquationf li;
    li.isValid = true;
    li.start.x = coefficients->values[0];
    li.start.y = coefficients->values[1];
    li.end.x   = li.start.x + coefficients->values[3];
    li.end.y   = li.start.y + coefficients->values[4];
    li.k       = (li.end.y - li.start.y) / (li.end.x - li.start.x);
    li.b       = li.end.y - li.k * li.end.x;
    li.inliers = inliers;
    //
    return li;
}

template<typename PointType>
sPlaneEquationf igeometry2<PointType>::planeFitting(PointsTypePtr inputCloud, float inlierThre) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(inlierThre);
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);
    if (0 == inliers->indices.size()) {
        sPlaneEquationf sp;
        sp.isValid = false;
        return sp;
    }
    sPlaneEquationf sp;
    sp.isValid = true;
    //ax+by+cz+1=0
    sp.a = coefficients->values[0] / coefficients->values[3];
    sp.b = coefficients->values[1] / coefficients->values[3];
    sp.c = coefficients->values[2] / coefficients->values[3];
    return sp;
}

template<typename PointType>
float igeometry2<PointType>::disPoint2Line(sPoint2Df pt, s2DLineEquationf line) {
    //Ax+By+C=0
    //A=k
    //B=-1
    //C=b
    //m=pt.x
    //n=pt.y
    //|Am+Bn+C|/sqrt(A^2+B^2)
    float dis1 = fabs(line.k * pt.x + (-1 * pt.y) + line.b);
    float dis2 = sqrt(line.k * line.k + 1);
    return dis1 / dis2;
}

template<typename PointType>
float igeometry2<PointType>::disPoint2Plane(PointType pt, sPlaneEquationf pl) {
    //ax+by+cz+1=0
    float dis1 = fabs(pl.a * pt.x + pl.b * pt.y + pl.c * pt.z + 1);
    float dis2 = sqrt(pl.a * pl.a + pl.b * pl.b + pl.c * pl.c);
    if(0 == dis2)
        return 0;
    return dis1 / dis2;
}
