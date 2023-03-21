#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>
#include "BetweenChordalFactor.h"

namespace Type
{
    using namespace gtsam;
    using namespace Eigen;
    using namespace std;
    static const Key keyAnchor = 99999999;

    Pose3 Eigen2Pose3(Vector3d translation, Quaterniond rotation);

    Rot3 Mat3d2Rot3(Quaterniond rotation);

    Vector changeToLinearizedRotation(Matrix3 R);
    
    Vector changeToLinearizedRotation(Pose3 pose);

    Values changeToLinearizedRotation(Values poseValues);

    VectorValues changeToLinearizedRotation(Values poseValues, bool ifVectorValues);

    Vector changeToLinearizedPose(Pose3 pose);

    Values changeToLinearizedPose(const Values &poseValues);

    GaussianFactorGraph buildLinearOrientationGraph(NonlinearFactorGraph graph);

    void updateValuesWithVectorValues(const VectorValues &input, Values &output);

    void updateValuesWithValues(const Values &input, Values &output);

    VectorValues Values2VectorValues(const Values &values);

    Values linearizedRotation2Pose(Values linearizedRotation);

    Values linearizedRotation2Pose(VectorValues linearizedRotation);

    NonlinearFactorGraph changeToChordalGraph(NonlinearFactorGraph &graph, 
                                              gtsam::noiseModel::Isotropic::shared_ptr model);

    Values retractToGlobalPose(const Values &initial, const Values &delta);

    Values retractToGlobalPose(const Values &initial, const VectorValues &delta);

} // namespace Type
