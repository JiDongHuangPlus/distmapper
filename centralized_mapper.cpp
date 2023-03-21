#include "centralized_mapper.h"
using namespace gtsam;
using namespace std;

void CentralizedMapper::addPriorFactor(const gtsam::Symbol &symbol, const gtsam::Pose3 &pose, const gtsam::SharedNoiseModel &model)
{
    NonlinearFactor::shared_ptr priorFactor(new PriorFactor<Pose3>(symbol.key(),
                                                                  pose,
                                                                  model));
    priorNoiseModel_ = noiseModel::Isotropic::Variance(6, 1e-12);
    graph_.add(priorFactor);
    addPose(symbol, pose);
    ifInitPrior_ = true;
    priorFlags_[symbol.chr()] = true;
}

void CentralizedMapper::addPose(const gtsam::Symbol &symbol, const gtsam::Pose3 &pose)
{
    initialPoses_.insert(symbol, pose);
    symbols_[symbol.chr()].push_back(symbol);
}

void CentralizedMapper::addPose(const gtsam::Symbol symbol, const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
    addPose(symbol, Type::Eigen2Pose3(translation, rotation));
}

void CentralizedMapper::addMeasure(const gtsam::Symbol &symbol1, const gtsam::Symbol &symbol2, const gtsam::Pose3 &relativePose)
{
    for(const boost::shared_ptr<NonlinearFactor>& factor: graph_)
    {
        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between = 
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
        if (pose3Between)
        {
            if (pose3Between->key1() == symbol1.key() && pose3Between->key2() == symbol2.key())
            {
                return;
            }
        }
    }
    BetweenFactor<Pose3> factor(symbol1.key(), symbol2.key(), relativePose, poseNoiseModel_);
    graph_.add(factor);
}

void CentralizedMapper::addMeasure(const gtsam::Symbol &symbol1, const gtsam::Symbol &symbol2, const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
    addMeasure(symbol1, symbol2, Type::Eigen2Pose3(translation, rotation));
}

gtsam::Values CentralizedMapper::estimate()
{
    auto chordalGraph = Type::changeToChordalGraph(graph_, poseNoiseModel_);
    cout << "initial: " << chordalGraph.error(initialPoses_) << endl;
    optimizeRotation();
    optimizePose();
    updateInitialPose();
    cout << "estimate: " << chordalGraph.error(initialPoses_) << endl;

    return initialPoses_;
}

void CentralizedMapper::optimizeRotation()
{
    linearizedRotation_ = Type::changeToLinearizedRotation(initialPoses_, true);
    auto linearizedGraph = Type::buildLinearOrientationGraph(graph_);
    auto newLinearizedRotation = linearizedGraph.optimize();
    updataRotationWithGamma(newLinearizedRotation);
    cout << "rotation: " << linearizedGraph.error(linearizedRotation_) << endl;
}

void CentralizedMapper::optimizePose()
{
    Values poseWithZeroTrans = Type::linearizedRotation2Pose(linearizedRotation_);
    auto chordalGraph = Type::changeToChordalGraph(graph_, poseNoiseModel_);
    Key firstKey = KeyVector(poseWithZeroTrans.keys()).at(0);
    chordalGraph.add(PriorFactor<Pose3>(firstKey, poseWithZeroTrans.at<Pose3>(firstKey), priorNoiseModel_));
    auto linearizedGraph = *(chordalGraph.linearize(poseWithZeroTrans));
    auto newLinearizedPose = linearizedGraph.optimize();
    updatePoseWithGamma(newLinearizedPose);
    cout << "pose: " << linearizedGraph.error(linearizedPose_) << endl;
}

void CentralizedMapper::updateInitialPose()
{
    Values poseWithZeroTrans = Type::linearizedRotation2Pose(linearizedRotation_);
    initialPoses_ = Type::retractToGlobalPose(poseWithZeroTrans, linearizedPose_);
}

void CentralizedMapper::updataRotationWithGamma(const gtsam::VectorValues &newLinearizedRotation)
{
    for (VectorValues::KeyValuePair& key_value: newLinearizedRotation)
    {
        Key key = key_value.first;
        if (linearizedRotation_.exists(key))
        {
            linearizedRotation_.at(key) = (1 - gamma_) * linearizedRotation_.at(key)
                + gamma_ * newLinearizedRotation.at(key);
        }
        else
        {
            linearizedRotation_.insert(key, newLinearizedRotation.at(key));
        }
    }
}

void CentralizedMapper::updatePoseWithGamma(const gtsam::VectorValues &newLinearizedPose)
{
    for (VectorValues::KeyValuePair& key_value: newLinearizedPose)
    {
        Key key = key_value.first;
        if (linearizedPose_.exists(key))
        {
            linearizedPose_.at(key) = (1 - gamma_) * linearizedPose_.at(key)
                + gamma_ * newLinearizedPose.at(key);
        }
        else
        {
            linearizedPose_.insert(key, newLinearizedPose.at(key));
        }
    }
}
