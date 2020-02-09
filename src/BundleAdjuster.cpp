// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include "BundleAdjuster.h"

BundleAdjuster::BundleAdjuster(Eigen::Vector3d camera_position_0, Eigen::AngleAxisd camera_orientation_0,
        Eigen::Vector3d camera_position_1, Eigen::AngleAxisd camera_orientation_1,
        Eigen::Vector3d camera_position_2, Eigen::AngleAxisd camera_orientation_2):
            camera_position_0(camera_position_0), camera_orientation_0(camera_orientation_0),
            camera_position_1(camera_position_1), camera_orientation_1(camera_orientation_1),
            camera_position_2(camera_position_2), camera_orientation_2(camera_orientation_2){}

bool BundleAdjuster::LoadFile(std::string filename) {
    FILE* fptr = fopen(filename.c_str(), "r");
    if (fptr == NULL) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_points_);

    point_index_ = new int[num_points_ * 3];
    camera_index_ = new int[num_points_ * 3];
    observations_ = new double[2 * num_points_ * 3];

    num_parameters_ = 6 + 3 * num_points_ + 3; //6 for new camera position and orientaiton, 3 for each obeservations, 3 for camera parameters
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_points_ * 3; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }

    double *camera = mutable_cameras();
    camera[0] = camera_orientation_2.angle() * camera_orientation_2.axis()[0];
    camera[1] = camera_orientation_2.angle() * camera_orientation_2.axis()[1];
    camera[2] = camera_orientation_2.angle() * camera_orientation_2.axis()[2];
    camera[3] = camera_position_2.x();
    camera[4] = camera_position_2.y();
    camera[5] = camera_position_2.z();

    for (int i = 0 ; i < num_points() ; ++i){
        double *point = mutable_points() + 3*i;
        point[0] = (std::rand() / (RAND_MAX + 1.)) * 2 - 1;
        point[1] = (std::rand() / (RAND_MAX + 1.)) * 2 - 1;
        point[2] = (std::rand() / (RAND_MAX + 1.)) * 2 - 1;
    }

    /*
    ceres::CostFunction* cost_function =
            IMUDifferenceError::Create(camera_position_0, camera_orientation_0);
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss * /,
                             mutable_cameras());*/

    for (int i = 0; i < num_observations(); ++i) {

        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        if (camera_index_[i] == 2){
            ceres::CostFunction* cost_function =
                    SnavelyReprojectionError::Create(observations()[2 * i + 0],
                                                     observations()[2 * i + 1]);
            problem.AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     mutable_cameras(),
                                     mutable_point_for_observation(i),
                                     mutable_camera_parameters());
        } else if(camera_index_[i] == 0){
            ceres::CostFunction* cost_function =
                    SnavelyReprojectionErrorFixedCamera::Create(observations()[2 * i + 0],
                                                                observations()[2 * i + 1],
                                                                camera_position_0, camera_orientation_0);
            problem.AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     mutable_point_for_observation(i),
                                     mutable_camera_parameters());
        } else if(camera_index_[i] == 1){
            ceres::CostFunction* cost_function =
                    SnavelyReprojectionErrorFixedCamera::Create(observations()[2 * i + 0],
                                                                observations()[2 * i + 1],
                                                                camera_position_1, camera_orientation_1);
            problem.AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     mutable_point_for_observation(i),
                                     mutable_camera_parameters());
        } else {
            std::cout << "More than 3 frames in keypoint file!!!" << std::endl;
        }

    }

    return true;
}

void BundleAdjuster::run(ceres::Solver::Summary* summary){
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 200;

    ceres::Solve(options, &problem, summary);
}

Eigen::Vector3d BundleAdjuster::getPosition() {
    double *camera = mutable_cameras();
    return Eigen::Vector3d(camera[3], camera[4], camera[5]);
}

Eigen::AngleAxisd BundleAdjuster::getOrientation() {
    double *camera = mutable_cameras();
    Eigen::Vector3d raw_orientation(camera[0], camera[1], camera[2]);
    return Eigen::AngleAxisd(raw_orientation.norm(), raw_orientation.normalized());
}


/*int main(int argc, char** argv) {

  const double* observations = bal_problem.observations();

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.

    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss * /,
                             bal_problem.mutable_camera_for_observation(i),
                             bal_problem.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}*/
