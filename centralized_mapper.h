#pragma once
#include <map>
#include <vector>
#include <algorithm>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Core>

#include "types.h"

class CentralizedMapper
{
    public:

    CentralizedMapper(){}
    void setNoiseModel(gtsam::noiseModel::Isotropic::shared_ptr poseNoiseModel,
                       gtsam::noiseModel::Diagonal::shared_ptr rotNoiseModel)
    {
        poseNoiseModel_ = poseNoiseModel;
        rotNoiseModel_ = rotNoiseModel;
    }
    bool isAddPrior(){ return ifInitPrior_;}
    bool isAddPrior(u_char name)
    {
        if(priorFlags_.find(name) != priorFlags_.end())
        {
            return priorFlags_[name];
        }
        else return false;
    }
    std::vector<gtsam::Symbol> getSymbolList(u_char name){ return symbols_[name];}

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

    gtsam::Values estimate();

    private:
    std::map<u_char, std::vector<gtsam::Symbol>> symbols_;
    std::map<u_char, bool> priorFlags_;
    double gamma_ = 1;
    gtsam::noiseModel::Isotropic::shared_ptr poseNoiseModel_ = nullptr;
    gtsam::noiseModel::Diagonal::shared_ptr rotNoiseModel_ = nullptr;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoiseModel_ = nullptr;
    bool ifInitPrior_ = false;

    gtsam::Values initialPoses_;
    gtsam::VectorValues linearizedRotation_;
    gtsam::VectorValues linearizedPose_;
    gtsam::NonlinearFactorGraph graph_;

    void optimizeRotation();
    void optimizePose();
    void updateInitialPose();
    void updataRotationWithGamma(const gtsam::VectorValues &newLinearizedRotation);
    void updatePoseWithGamma(const gtsam::VectorValues &newLinearizedPose);


};