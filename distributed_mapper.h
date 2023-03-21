#pragma once
#include <map>
#include <vector>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Core>

#include "types.h"

class DistributedMapper
{
    public:
    DistributedMapper();
    void setRobotInfo(int16_t seq, unsigned char name, double gamma = 1)
    {
        seq_ = seq;
        name_ = name;
        gamma_ = gamma;
    }
    void setNoiseModel(gtsam::noiseModel::Isotropic::shared_ptr poseNoiseModel,
                       gtsam::noiseModel::Diagonal::shared_ptr rotNoiseModel)
    {
        poseNoiseModel_ = poseNoiseModel;
        rotNoiseModel_ = rotNoiseModel;
    }
    uint64_t getPoseCount(){ return poseCount_;}
    unsigned char getName(){ return name_;}
    bool isCurrentRobot(gtsam::Symbol symbol)
    {
        return symbol.chr() == name_ ? true : false;
    }
    bool isAddPrior(){ return ifInitPrior_;}
    gtsam::Pose3 getPoseAt(gtsam::Symbol symbol)
    {
        return currentRobotPoses_.at<gtsam::Pose3>(symbol.key());
    }
    gtsam::Pose3 getPoseAt(gtsam::Key key)
    {
        return currentRobotPoses_.at<gtsam::Pose3>(key);
    }
    std::vector<std::uint64_t> getNeighborKeys(unsigned char name)
    {
        return neighborPoseSymbol_[name];
    }
    gtsam::Values getCurrentRobotPose(){ return currentRobotPoses_;}

    void addPriorFactor(const gtsam::Symbol &symbol,
                        const gtsam::Pose3 &pose,
                        const gtsam::SharedNoiseModel& model);
    void addPose(const gtsam::Symbol &symbol, 
                 const gtsam::Pose3 &pose);
    void addPose(const gtsam::Symbol symbol,
                 const Eigen::Vector3d &translation,
                 const Eigen::Quaterniond &rotation);
    void addMeasure(const gtsam::Symbol &symbol1, 
                    const gtsam::Symbol &symbol2, 
                    const gtsam::Pose3 &relativePose);
    void addMeasure(const gtsam::Symbol &symbol1, 
                    const gtsam::Symbol &symbol2, 
                    const Eigen::Vector3d &translation,
                    const Eigen::Quaterniond &rotation);
    void updataNeighborPoses(const gtsam::Symbol &symbol,
                             const gtsam::Pose3 &pose);
    
    gtsam::Values estimate();

    public:
    /** Robot Information */
    int16_t seq_;
    unsigned char name_;
    uint64_t poseCount_ = 0;
    double gamma_ = 1;
    bool ifInitPrior_ = false;//if this robot is initialized with prior factor
    gtsam::noiseModel::Isotropic::shared_ptr poseNoiseModel_ = nullptr;
    gtsam::noiseModel::Diagonal::shared_ptr rotNoiseModel_ = nullptr;

    /** Graph for optimization and cache*/
    gtsam::NonlinearFactorGraph currentPoseGraph_;//
    gtsam::NonlinearFactorGraph fullPoseGraph_;//
    std::vector<size_t> separatorEdgeIds_;//
    std::map<unsigned char, std::vector<std::uint64_t>> neighborPoseSymbol_;

    /** Values for current robot and neigborhood robots*/
    gtsam::Values fullRobotsPoses_;
    gtsam::Values currentRobotPoses_;
    gtsam::Values neighborRobotsPoses_;
    gtsam::Values linearizedRotation_;
    gtsam::Values neighborLinearizedRotation_;
    gtsam::Values linearizedPoses_;
    gtsam::Values neighborLinearizedPoses_;

    void optimizeRotation();
    void optimizePose();
    void updateCurrentPoses();

    void addNeighborRotationAsPrior(gtsam::GaussianFactorGraph &linearizedGraph);
    void addNeighborPosesAsPrior(gtsam::GaussianFactorGraph &linearizedGraph);
    void updataRotationWithGamma(const gtsam::Values &newLinearizedRotation);
    void updatePoseWithGamma(const gtsam::Values &newLinearizedPose);
};
