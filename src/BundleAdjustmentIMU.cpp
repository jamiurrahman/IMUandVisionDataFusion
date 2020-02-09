//
// Created by jochen on 1/20/20.
//

#include "BundleAdjustmentIMU.h"

BundleAdjustmentIMU::BundleAdjustmentIMU(const char *imuFile, const char *keypointFile) : bundleAdjuster(), uncorrectedImuSteps {} {
    //load imu data
    std::ifstream infile(imuFile);
    std::string line = "";

    std::string row[12];

    int i = 0;

    getline(infile,line); // discard first line

    while (getline(infile, line)){

        std::stringstream strstr(line);
        std::string word = "";
        int j = 0;
        while (getline(strstr,word, ',')){
            row[j++] = word;
        }

        //std::cout<<row[5]<<","<<row[6]<<","<<row[7]<<","<<std::endl;
        Eigen::Vector3d deltaPosition(stod(row[5]),stod(row[6]),stod(row[7]));
        Eigen::Vector3d deltaOrientation(stod(row[9]),stod(row[10]),stod(row[11]));

        uncorrectedImuSteps.push_back(ImuStep(++i,deltaPosition,deltaOrientation));
    }

    //load keypoints
    if (!bundleAdjuster.LoadFile(keypointFile, false)){
        std::cout << "Keypoint file not found" << std::endl;
        return;
    }

    //add first frame
    bundleAdjuster.addFrame(0, Eigen::Vector3d(0,0,0), Eigen::AngleAxisd::Identity());
    lastFrame = 0;
}

bool BundleAdjustmentIMU::runStep(ceres::Solver::Summary* summary) {
    if (uncorrectedImuSteps.empty()){
        return false;
    }

    //get velocity at start
    Eigen::Vector3d lastFrameVel(0,0,0);
    Eigen::Vector3d lastFramePos = bundleAdjuster.getPosition(lastFrame);
    if (lastFrame > 0){
        lastFrameVel = (lastFramePos - bundleAdjuster.getPosition(lastFrame - 1)) / 15.0;
    }
    ImuStep imuStep = uncorrectedImuSteps.front();
    uncorrectedImuSteps.pop_front();

    Eigen::AngleAxisd rollAngle(imuStep.deltaOrientation[0] / 15.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(imuStep.deltaOrientation[1] / 15.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(imuStep.deltaOrientation[2] / 15.0, Eigen::Vector3d::UnitZ());

    Eigen::AngleAxisd newOrientation(rollAngle * pitchAngle * yawAngle * bundleAdjuster.getOrientation(lastFrame));

    bundleAdjuster.addFrame(++lastFrame, lastFramePos + (lastFrameVel + imuStep.acceleration / 15.0) / 15.0, newOrientation);

    bundleAdjuster.run(summary);
    return true;
}

void BundleAdjustmentIMU::saveModel(const char *modelFile) {
    std::ofstream outFile(modelFile);
    if (!outFile.is_open()) return;

    // Write header.
    outFile << "COFF" << std::endl;
    outFile << (lastFrame + 1) << " 0 0" << std::endl;

    // Save vertices.
    for (int i = 0 ; i<=lastFrame ; i++) {
        Eigen::Vector3d pos = bundleAdjuster.getPosition(i);
        outFile << pos.x() << " " << pos.y() << " " << pos.z() << " 255 255 0 255" << std::endl;
    }

    // Close file.
    outFile.close();

    return;
}