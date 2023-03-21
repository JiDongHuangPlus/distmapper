#include "types.h"
using namespace gtsam;
using namespace Eigen;
using namespace std;

Pose3 Type::Eigen2Pose3(Vector3d translation, Quaterniond rotation)
{
    return Pose3(Rot3(rotation.matrix()), Point3(translation));
}

Rot3 Type::Mat3d2Rot3(Quaterniond rotation)
{
    return Rot3(rotation.matrix());
}

Vector Type::changeToLinearizedRotation(Matrix3 R)
{
    return (Vector(9) << R(0,0), R(0,1), R(0,2),
                         R(1,0), R(1,1), R(1,2),
                         R(2,0), R(2,1), R(2,2)).finished();;
}

Vector Type::changeToLinearizedRotation(Pose3 pose)
{
    return changeToLinearizedRotation(pose.rotation().matrix());
}

Values Type::changeToLinearizedRotation(Values poseValues)
{
    Values values;
    for (const Values::ConstKeyValuePair& key_value: poseValues)
    {
        Pose3 pose = poseValues.at<Pose3>(key_value.key);
        Vector linearizedRotation = changeToLinearizedRotation(pose);
        values.insert(key_value.key, linearizedRotation);
    }
    return values;
}

VectorValues Type::changeToLinearizedRotation(Values poseValues, bool ifVectorValues)
{
    VectorValues values;
    for (const Values::ConstKeyValuePair& key_value: poseValues)
    {
        Pose3 pose = poseValues.at<Pose3>(key_value.key);
        Vector linearizedRotation = changeToLinearizedRotation(pose);
        values.insert(key_value.key, linearizedRotation);
    }
    return values;
}

Vector Type::changeToLinearizedPose(Pose3 pose)
{
    Vector3 so3 = Rot3::Logmap(pose.rotation());
    Vector3 t = pose.translation().matrix();
    return (Vector(6) << so3[0], so3[1], so3[2],
                         t[0], t[1], t[2]).finished();
}

Values Type::changeToLinearizedPose(const Values &poseValues)
{
    Values values;
    for (const Values::ConstKeyValuePair& key_value: poseValues)
    {
        Pose3 pose = poseValues.at<Pose3>(key_value.key);
        values.insert(key_value.key, changeToLinearizedPose(pose));
    }
    return values;
}

GaussianFactorGraph Type::buildLinearOrientationGraph(NonlinearFactorGraph graph)
{
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graph);
    GaussianFactorGraph linearGraph;
    SharedDiagonal model = noiseModel::Unit::Create(9);

    for (const boost::shared_ptr<NonlinearFactor>& factor: pose3Graph)
    {
        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between = 
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
        auto Rij = pose3Between->measured().rotation().matrix();
        const FastVector<Key>& keys = factor->keys();
        Key key1 = keys[0], key2 = keys[1];
        gtsam::Matrix M9 = Z_9x9;
        M9.block<3, 3>(0, 0) = Rij;
        M9.block<3, 3>(3, 3) = Rij;
        M9.block<3, 3>(6, 6) = Rij;
        linearGraph.add(key1, -I_9x9, key2, M9, Z_9x1, model);
    }
    // prior on the anchor orientation
    gtsam::Vector e = Vector(9);
    e << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    linearGraph.add(keyAnchor, I_9x9, e, model);
    return linearGraph;
}

void Type::updateValuesWithVectorValues(const VectorValues &input, Values &output)
{
    for (VectorValues::KeyValuePair key_value: input)
    {
        Key key = key_value.first;
        if (key == keyAnchor) continue;
        if (output.exists(key))
        {
            output.update(key, input.at(key));
        }
        else
        {
            // Symbol symbol(key);
            // cerr << "Warning: " << symbol.chr() << symbol.index()
            // << " cannot be updates" << endl;
            output.insert(key, input.at(key));
        }
    }
}

void Type::updateValuesWithValues(const Values &input, Values &output)
{
    for (const Values::ConstKeyValuePair& key_value: input)
    {
        Key key = key_value.key;
        if (output.exists(key))
        {
            output.update(key, input.at(key));
        }
        else
        {
            output.insert(key, input.at(key));
        }
    }
}

VectorValues Type::Values2VectorValues(const Values &values)
{
    VectorValues output;
    for (const Values::ConstKeyValuePair& key_value: values)
    {
        Key key = key_value.key;
        output.insert(key, values.at<Vector>(key));
    }
    return output;
}

Values Type::linearizedRotation2Pose(Values linearizedRotation)
{
    return linearizedRotation2Pose(Values2VectorValues(linearizedRotation));
}

Values Type::linearizedRotation2Pose(VectorValues linearizedRotation)
{
    Values rotInSO3 = InitializePose3::normalizeRelaxedRotations(linearizedRotation);
    Values poses;
    for(const Values::ConstKeyValuePair& key_value: rotInSO3)
    {
        Key key = key_value.key;
        Pose3 pose(rotInSO3.at<Rot3>(key), Point3(0, 0, 0));
        poses.insert(key, pose);
    }
    return poses;
}

NonlinearFactorGraph Type::changeToChordalGraph(NonlinearFactorGraph &graph, gtsam::noiseModel::Isotropic::shared_ptr model)
{
    NonlinearFactorGraph chordalGraph;
    //graph = InitializePose3::buildPose3graph(graph);
    for(size_t k = 0; k < graph.size(); k++)
    {
        boost::shared_ptr<BetweenFactor<Pose3> > factor =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
        if(factor)
        {
            Key key1 = factor->key1();
            Key key2 = factor->key2();
            Pose3 measured = factor->measured();
            chordalGraph.add(BetweenChordalFactor<Pose3>(key1, key2, measured, model));
        }
        else
        {
            //chordalGraph.add(graph[k]);
        }
    }
    return chordalGraph;
}

Values Type::retractToGlobalPose(const Values &initial, const Values &delta)
{
    Values estimate;
    for(const Values::ConstKeyValuePair& key_value: initial)
    {
        Key key = key_value.key;
        Vector deltaVec = delta.at<Vector>(key);
        Pose3 initialPose = initial.at<Pose3>(key);
        Rot3 R = initialPose.rotation().retract(deltaVec.block<3, 1>(0, 0));
        Point3 t = initialPose.translation() + Point3(deltaVec.block<3, 1>(3, 0));
        estimate.insert(key, Pose3(R, t));
    }
    return estimate;
}

Values Type::retractToGlobalPose(const Values &initial, const VectorValues &delta)
{
    Values estimate;
    for(const Values::ConstKeyValuePair& key_value: initial)
    {
        Key key = key_value.key;
        Vector6 deltaVec = delta.at(key);
        Pose3 initialPose = initial.at<Pose3>(key);
        Rot3 R = initialPose.rotation().retract(deltaVec.head(3));
        Point3 t = initialPose.translation() + Point3(deltaVec.tail(3));
        estimate.insert(key, Pose3(R, t));
    }
    return estimate;
}
