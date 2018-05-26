/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * author: wangxiao
 * update log:
 * 20170501:
 * 1) make up the basic static algorithm framework
*/
#ifndef CLASS_GEOMETRYWITHI2_H
#define CLASS_GEOMETRYWITHI2_H
#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "common.h"

template<typename PointType>
class igeometry2
{
public:
    explicit igeometry2();
    virtual ~igeometry2(){};

    typedef typename pcl::PointCloud<PointType> PointsType;
    typedef typename PointsType::Ptr PointsTypePtr;

    s2DLineEquationf lineFitting(PointsTypePtr input, float inlierThre);
    sPlaneEquationf planeFitting(PointsTypePtr inputCloud, float inlierThre);
    float disPoint2Line(sPoint2Df pt, s2DLineEquationf line);
    float disPoint2Plane(PointType pt, sPlaneEquationf pl);

};
#endif
