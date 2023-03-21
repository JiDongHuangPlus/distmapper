#include "distributed_mapper.h"
using namespace gtsam;
using namespace std;

DistributedMapper::DistributedMapper()
{

}

void DistributedMapper::addPriorFactor(const gtsam::Symbol &symbol, const gtsam::Pose3 &pose, const gtsam::SharedNoiseModel &model)
{
    NonlinearFactor::shared_ptr poseFactor(new PriorFactor<Pose3>(symbol.key(),
                                                                  pose,
                                                                  model));
    fullPoseGraph_.add(poseFactor);
    currentPoseGraph_.add(poseFactor);
    addPose(symbol, pose);

    ifInitPrior_ = true;
}

void DistributedMapper::addPose(const gtsam::Symbol &symbol, const gtsam::Pose3 &pose)
{
    fullRobotsPoses_.insert(symbol, pose);
    if (symbol.chr() == name_)
    {
        currentRobotPoses_.insert(symbol, pose);
        poseCount_++;
    } 
    else
    {
        neighborRobotsPoses_.insert(symbol, pose);
        auto name = symbol.chr();
        if (neighborPoseSymbol_.find(name) == neighborPoseSymbol_.end())
        {
            vector<std::uint64_t> keys;
            keys.push_back(symbol.key());
            neighborPoseSymbol_[name] = keys;
        }
        else
        {
            neighborPoseSymbol_[name].push_back(symbol.key());
        }
    }
}

void DistributedMapper::addPose(const gtsam::Symbol symbol, const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
    addPose(symbol, Type::Eigen2Pose3(translation, rotation));
}

void DistributedMapper::addMeasure(const gtsam::Symbol &symbol1, const gtsam::Symbol &symbol2, const gtsam::Pose3 &relativePose)
{
    BetweenFactor<Pose3> factor(symbol1.key(), symbol2.key(), relativePose, poseNoiseModel_);
    fullPoseGraph_.add(factor);
    if (symbol1.chr() == name_ && symbol2.chr() == name_)
    {
        currentPoseGraph_.add(factor);
    }
    else
    {
        if (symbol1.chr() != name_ && symbol2.chr() != name_)
        {
            cerr << "Warning: measurement between " << symbol1.chr() << symbol1.index()
            << " and " << symbol2.chr() << symbol2.index() << " should be from current robot "
            << name_ << endl;
        }
        else 
        {
            separatorEdgeIds_.push_back(fullPoseGraph_.size() - 1);
            if (symbol1.chr() != name_)
            {
                neighborPoseSymbol_[symbol1.chr()].push_back(symbol1.key());
            }
            else
            {
                neighborPoseSymbol_[symbol2.chr()].push_back(symbol2.key());
            }
        }
    }
}

void DistributedMapper::addMeasure(const gtsam::Symbol &symbol1, const gtsam::Symbol &symbol2, const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
    addMeasure(symbol1, symbol2, Type::Eigen2Pose3(translation, rotation));
}

void DistributedMapper::updataNeighborPoses(const gtsam::Symbol &symbol, const gtsam::Pose3 &pose)
{
    if (neighborRobotsPoses_.exists(symbol))
    {
        neighborRobotsPoses_.update(symbol, pose);
    }
    else
    {
        addPose(symbol, pose);
    }
}

Values DistributedMapper::estimate()
{
    optimizeRotation();
    optimizePose();
    updateCurrentPoses();
    return currentRobotPoses_;
}

void DistributedMapper::optimizeRotation()
{
    linearizedRotation_ = Type::changeToLinearizedRotation(currentRobotPoses_);
    auto linearizedGraph = Type::buildLinearOrientationGraph(currentPoseGraph_);
    addNeighborRotationAsPrior(linearizedGraph);
    VectorValues result = linearizedGraph.optimize();
    Values newLinearizedRotation;
    Type::updateValuesWithVectorValues(result, newLinearizedRotation);
    updataRotationWithGamma(newLinearizedRotation);
}

void DistributedMapper::optimizePose()
{
    Values initialPose = Type::linearizedRotation2Pose(linearizedRotation_);
    auto chordalGraph = Type::changeToChordalGraph(currentPoseGraph_, poseNoiseModel_);
    auto linearizedGraph = *(chordalGraph.linearize(initialPose));
    addNeighborPosesAsPrior(linearizedGraph);
    VectorValues result = linearizedGraph.optimize();
    Values newLinearizedPose;
    Type::updateValuesWithVectorValues(result, newLinearizedPose);
    updatePoseWithGamma(newLinearizedPose);
}

void DistributedMapper::updateCurrentPoses()
{  
    Values initialPose = Type::linearizedRotation2Pose(linearizedRotation_);
    currentRobotPoses_ = Type::retractToGlobalPose(initialPose, linearizedPoses_);
    Type::updateValuesWithValues(currentRobotPoses_, fullRobotsPoses_);
}

void DistributedMapper::addNeighborRotationAsPrior(GaussianFactorGraph &linearizedGraph)
{
    if (neighborRobotsPoses_.empty()) return;
    for (auto sep: separatorEdgeIds_)
    {
        boost::shared_ptr<BetweenFactor<Pose3>> factor =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(fullPoseGraph_.at(sep));
        Pose3 relativePose = factor->measured();
        Matrix3 R01t = relativePose.rotation().transpose().matrix();
        Matrix M9 = Z_9x9;
        M9.block(0,0,3,3) = R01t;
        M9.block(3,3,3,3) = R01t;
        M9.block(6,6,3,3) = R01t;
        auto name0 = symbolChr(factor->key1());
        auto name1 = symbolChr(factor->key2());
        if (name0 == name_)
        {
            Pose3 R1 = neighborRobotsPoses_.at(factor->key2()).cast<Pose3>();
            Vector r1 = Type::changeToLinearizedRotation(R1);
            linearizedGraph.add(factor->key1(), M9, r1, rotNoiseModel_);
        }
        else
        {
            Pose3 R0 = neighborRobotsPoses_.at(factor->key1()).cast<Pose3>();
            Vector r0 = Type::changeToLinearizedRotation(R0);
            Vector M9_r0 = M9 * r0;
            linearizedGraph.add(factor->key2(), I_9x9, M9_r0, rotNoiseModel_);
        }
    }
}

void DistributedMapper::addNeighborPosesAsPrior(gtsam::GaussianFactorGraph &linearizedGraph)
{
    if (neighborRobotsPoses_.empty()) return;
    for (auto sep: separatorEdgeIds_)
    {
        boost::shared_ptr<BetweenFactor<Pose3>> factor =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(fullPoseGraph_.at(sep));
        Pose3 relativePose = factor->measured();
        Matrix M0 = Matrix::Zero(12,6);
        Matrix M1 = Matrix::Zero(12,6);
        auto name0 = symbolChr(factor->key1());
        auto name1 = symbolChr(factor->key2());
        BetweenChordalFactor<Pose3> betweenChordalFactor(factor->key1(), 
                                                         factor->key2(), 
                                                         relativePose, 
                                                         poseNoiseModel_);
        if (name0 == name_)
        {
            auto currentPose = currentRobotPoses_.at<Pose3>(factor->key1());
            auto neighborPose = neighborRobotsPoses_.at<Pose3>(factor->key2());
            Vector error = betweenChordalFactor.evaluateError(currentPose, neighborPose, M0, M1);
            Matrix A = M0;
            Vector b = -(M1 * Type::changeToLinearizedPose(neighborPose) + error);
            linearizedGraph.add(factor->key1(), A, b, poseNoiseModel_);
        }
        else
        {
            auto currentPose = currentRobotPoses_.at<Pose3>(factor->key2());
            auto neighborPose = neighborRobotsPoses_.at<Pose3>(factor->key1());
            Vector error = betweenChordalFactor.evaluateError(neighborPose, currentPose, M0, M1);
            Matrix A = M1;
            Vector b = -(M0 * Type::changeToLinearizedPose(neighborPose) + error);
            linearizedGraph.add(factor->key2(), A, b, poseNoiseModel_);
        }
    }
}

void DistributedMapper::updataRotationWithGamma(const gtsam::Values &newLinearizedRotation)
{
    for(const gtsam::Values::ConstKeyValuePair& key_value: newLinearizedRotation)
    {
        Key key = key_value.key;
        if(!linearizedRotation_.exists(key))
        {
            linearizedRotation_.insert(key, newLinearizedRotation.at(key));
        }
        else
        {
            Vector newRotation = (1 - gamma_) * linearizedRotation_.at<Vector>(key)
            + gamma_ * newLinearizedRotation.at<Vector>(key);
            linearizedRotation_.update(key, newRotation);
        }
    }
}

void DistributedMapper::updatePoseWithGamma(const gtsam::Values &newLinearizedPose)
{
    for(const gtsam::Values::ConstKeyValuePair& key_value: newLinearizedPose)
    {
        Key key = key_value.key;
        if(!linearizedPoses_.exists(key))
        {
            linearizedPoses_.insert(key, newLinearizedPose.at(key));
        }
        else
        {
            Vector newPose = (1 - gamma_) * linearizedPoses_.at<Vector>(key)
            + gamma_ * newLinearizedPose.at<Vector>(key);
            linearizedPoses_.update(key, newPose);
        }
    }
}
