#pragma once
#include <Eigen/Dense>
#include <list>
#include <stdexcept>

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include "CameraState.h"

class IMUIntegration {
public:
    /**
     * Initializes the IMUINtegration with default initial cameraState (position 0,0,0; oritentation 0;0;0)
     */
    IMUIntegration(float framerate);

    /**
     * Initializes the IMUIntegration with a given initial cameraState
     * @param initalCameraState initial cmaeraState
     */
    IMUIntegration(float framerate, CameraState initalCameraState);


    /**
     * Initializes the IMUIntegration with imu data from csv file given and default initial cameraState (position 0,0,0; oritentation 0;0;0)
     * @param filename cvs file to read imu data from
     */
    IMUIntegration(float framerate, std::string filename);

    /**
     * Initializes the IMUIntegration with imu data from csv file given and initial cameraState
     * @param initalCameraState  initial cmaeraState
     * @param filename cvs file to read imu data from
     */
    IMUIntegration(float framerate, CameraState initalCameraState, std::string filename);

    /**
     * Adds a new IMU meassurement. The meassurements have to be added in order
     * @param imuStep step to be added
     */
    void addImuStep(ImuStep imuStep);

    /**
     * gets the corrected camera state for a given time
     * @param time the frame for witch the camera state is to be calculated
     * @return the corrected camera state at the given time
     */
    CameraState getCameraState(int frame);

    /**
     * @return the next camera state to be corrected
     */
    CameraState getNextCameraState();

    /**
     * @return weather there is a uncorrected CameraState left
     */
    bool hasNextCameraState();

    /**
     * Corrects the next camera state with new camera state
     * @param the corrected cameraState
     */
    void correctCameraState(CameraState cameraState);

    /**
     * Saves the path as a 3d mesh
     * @param file to save the mesh to
     */
    void saveModel(std::string filename);

private:

    struct PathState{
        PathState(int _frame, CameraState _cameraState, Eigen::Vector3d _velocity):
                frame(_frame), cameraState(_cameraState), velocity(_velocity) {};

        CameraState cameraState;
        Eigen::Vector3d velocity;
        int frame;
    };

    float deltaTime;
    Eigen::Vector3d correctedVel;
    CameraState savedNextState;

    std::list<ImuStep> uncorrectedImuSteps; //orderd list of ImuSteps after the last path position
    std::list<PathState> path; //ordered list of corrected camera states
};