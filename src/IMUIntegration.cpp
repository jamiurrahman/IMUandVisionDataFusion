#include "IMUIntegration.h"


IMUIntegration::IMUIntegration(float framerate) : IMUIntegration(framerate, CameraState(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity())) {}

IMUIntegration::IMUIntegration(float framerate, CameraState initalPathState) : path {} {
    path.push_back(PathState(0, initalPathState, Eigen::Vector3d(0,0,0)));
    deltaTime = 1/framerate;
}

IMUIntegration::IMUIntegration(float framerate, std::string filename) : IMUIntegration(framerate, CameraState(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity()), filename) {}
IMUIntegration::IMUIntegration(float framerate, CameraState initalCameraState, std::string filename) : IMUIntegration(framerate, initalCameraState){
    std::ifstream infile(filename);
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

        addImuStep(ImuStep(++i,deltaPosition,deltaOrientation));

    }
}



void IMUIntegration::addImuStep(ImuStep imuStep) {
    if (path.back().frame > imuStep.frame){
        throw std::invalid_argument( "new imuStep is older than imuStep alreaddy added" );
        return;
    }

    uncorrectedImuSteps.push_back(imuStep);
}

CameraState IMUIntegration::getCameraState(int frame) {
    for (auto &state : path){
        if (state.frame == frame){
            return state.cameraState;
        }
    }

    return CameraState(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond());
}


bool IMUIntegration::hasNextCameraState(){
    return !uncorrectedImuSteps.empty();
}

CameraState IMUIntegration::getNextCameraState(){
    PathState& lastPathStep = path.back();
    ImuStep& imuStep = uncorrectedImuSteps.front();


    Eigen::AngleAxisd rollAngle(imuStep.deltaOrientation[0],Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(imuStep.deltaOrientation[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(imuStep.deltaOrientation[2],Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond newOrientation = rollAngle * pitchAngle * yawAngle * lastPathStep.cameraState.orientation;

    auto euler = newOrientation.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << euler << std::endl << std::endl;

    Eigen::Vector3d gravity(0.0,0.0,0.0);

    Eigen::Vector3d newPosition = lastPathStep.cameraState.position + lastPathStep.velocity + (newOrientation * imuStep.acceleration + gravity) * deltaTime/ 2.0 ;

    correctedVel = lastPathStep.velocity + newOrientation * imuStep.acceleration * deltaTime;

    savedNextState = CameraState(newPosition, newOrientation);
    return savedNextState;
}

void IMUIntegration::correctCameraState(CameraState cameraState) {
    PathState& lastPathStep = path.back();

//    Eigen::Vector3d correctedVelocity = correctedVel;

    if ((cameraState.position - savedNextState.position).norm() > 5.0){
        path.push_back(PathState(lastPathStep.frame + 1, savedNextState, correctedVel));
        std::cout << "Updated position too different, only using IMU data" << std::endl;
    } else {
        Eigen::Vector3d correctedVelocity = correctedVel;// * 0.9 + (2 * cameraState.position - 2 * lastPathStep.cameraState.position - lastPathStep.velocity) * 0.1;
        CameraState newState(cameraState.position * 0.1 + savedNextState.position * 0.9,
                             cameraState.orientation.slerp(0.1, savedNextState.orientation));

        path.push_back(PathState(lastPathStep.frame + 1, newState, correctedVelocity));
    }

    uncorrectedImuSteps.pop_front();
}

void IMUIntegration::saveModel(std::string filename){
    // Write off file.
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return;

    // Write header.
    outFile << "COFF" << std::endl;
    outFile << path.size() << " 0 0" << std::endl;

    // Save vertices.
    for (auto &state : path) {
        outFile << state.cameraState.position.x() << " " << state.cameraState.position.y() << " " << state.cameraState.position.z() << " 255 255 0 255" << std::endl;
    }

    // Close file.
    outFile.close();

    return;
}

