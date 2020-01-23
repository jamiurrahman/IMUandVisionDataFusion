#include "IMUIntegration.h"

int main()
{
    IMUIntegration imuIntegration(15.0, "../data/20191218_100925.csv");

    int i = 0;

    while(imuIntegration.hasNextCameraState()){
        i++;

        CameraState nextCameraState = imuIntegration.getNextCameraState();

        //Test of correcting nextCameraState (note, position updates influences velocity; orientation update also updates orientation of accelerations).
        if (i==100){
            //nextCameraState.position += Eigen::Vector3d(20,0,0);
            nextCameraState.orientation = Eigen::AngleAxisd(0.5*M_PI,Eigen::Vector3d::UnitY()) * nextCameraState.orientation;
        }

        imuIntegration.correctCameraState(nextCameraState);
    }

    imuIntegration.saveModel("../out/20191218_100925.off");
}