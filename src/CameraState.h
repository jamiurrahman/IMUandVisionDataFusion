#pragma once
#include <Eigen/Dense>

struct CameraState{
    CameraState() {};
    CameraState(Eigen::Vector3d _position, Eigen::Quaterniond _orientation):
            position(_position), orientation(_orientation) {};
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation; //as quaternion to avoid problems with euler angles
};

struct ImuStep{
    ImuStep(int _frame, Eigen::Vector3d _acceleration, Eigen::Vector3d _deltaOrientation):
            frame(_frame), acceleration(_acceleration), deltaOrientation(_deltaOrientation) {};
    int frame;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d deltaOrientation; //euler angles as reported by IMU
};