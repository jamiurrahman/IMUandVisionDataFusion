#include "IMUIntegration.h"
#include "BundleAdjuster.h"
#include "BundleAdjustmentIMU.h"

int main()
{
    /*
    BundleAdjustmentIMU baimu("../data/20200117_184501.csv", "../data/20200117_184501.matches.init.txt");

    int i = 0;
    ceres::Solver::Summary summary;
    while (baimu.runStep(&summary)){
        i++;
        std::cout << "Step " << i << " completed" << std::endl;
        std::cout << summary.BriefReport() << std::endl;
        std::cout << "========================================" << std::endl;
    }

    baimu.saveModel("../out/20191218_100925.off");*/

   IMUIntegration imuIntegration(10.0, "../data/straight_movement/imu.csv");

    int i = 0;

    while(imuIntegration.hasNextCameraState()){
        i++;
        std::cout << "Step " << i << std::endl << std::endl;

        //get next camera state
        CameraState nextCameraState = imuIntegration.getNextCameraState();

        if (i >= 2){ // dont adjust first camera state and use imu position
            //get last two frames
            CameraState secondlastframe = imuIntegration.getCameraState(i-2);
            CameraState lastframe = imuIntegration.getCameraState(i-1);

            //call Bundle adjustment with three frames
            BundleAdjuster ba(
                    secondlastframe.position, Eigen::AngleAxisd(secondlastframe.orientation),
                    lastframe.position,  Eigen::AngleAxisd(lastframe.orientation),
                    nextCameraState.position,  Eigen::AngleAxisd(nextCameraState.orientation)
                    );

            std::string filename = "../data/straight_movement/keypoint_";
            filename.append(std::to_string(i)).append(".txt");
            if (!ba.LoadFile(filename)) { //load keypoints and create residual terms
                std::cout << "file " << filename << " not found" << std::endl;
                break;
            } else {

                ceres::Solver::Summary summary;
                ba.run(&summary); // run solver
                std::cout << summary.BriefReport() << std::endl; //print bried report

                std::cout << "old position: " << nextCameraState.position.x() << ", " << nextCameraState.position.y() << ", " << nextCameraState.position.z() << std::endl;
                if (summary.IsSolutionUsable()) {
                    //get position of current frame
                    nextCameraState.position = ba.getPosition();
                    std::cout << "new position: " << nextCameraState.position.x() << ", " << nextCameraState.position.y() << ", " << nextCameraState.position.z() << std::endl;
                    nextCameraState.orientation = Eigen::Quaterniond(ba.getOrientation());
                }
            }
        }

        imuIntegration.correctCameraState(nextCameraState);



        std::cout << "=========================================" << std::endl;
    }

    imuIntegration.saveModel("../out/straight_movement.off");
}