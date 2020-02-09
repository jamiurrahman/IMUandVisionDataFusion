#pragma once

#include "CameraState.h"
#include "BundleAdjuster.h"

#include <list>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


class BundleAdjustmentIMU {
public:
    BundleAdjustmentIMU(const char* imuFile, const char* keypointFile);
    bool runStep(ceres::Solver::Summary* summary);
    void saveModel(const char* modelFile);
private:
    BundleAdjuster bundleAdjuster;
    std::list<ImuStep> uncorrectedImuSteps;
    int lastFrame;
};
