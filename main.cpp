#include "distributed_mapper.h"
#include "centralized_mapper.h"
#include <gtsam/slam/dataset.h>
#include <boost/lexical_cast.hpp>
#include <string>
using namespace gtsam;
using namespace std;

int testOneRobot();
int testTwoRobots();
int centralized();

int main()
{
    //testOneRobot();
    //testTwoRobots();
    centralized();
    
    return 0;
}

int testOneRobot()
{
    DistributedMapper distMapper[2];
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12);
    //
    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        auto initial = *(graphAndValue.second);
        auto name = Symbol(initial.keys()[0]).chr();
        distMapper[i].setRobotInfo(i, name);
    }

    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        
        noiseModel::Diagonal::shared_ptr rotModel = noiseModel::Isotropic::Variance(9, 1);
        noiseModel::Isotropic::shared_ptr posModel = noiseModel::Isotropic::Variance(12, 1);
        distMapper[i].setNoiseModel(posModel, rotModel);

        for (const Values::ConstKeyValuePair& key_value: *(graphAndValue.second))
        {
            Key key = key_value.key;
            Pose3 pose = key_value.value.cast<Pose3>();
            if(distMapper[i].isAddPrior())
            {
                distMapper[i].addPose(Symbol(key), pose);
            }
            else
            {
                distMapper[i].addPriorFactor(Symbol(key), pose, priorModel);
            }
        }
    }
    
    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        for (const boost::shared_ptr<NonlinearFactor>& factor: *(graphAndValue.first))
        {
            boost::shared_ptr<BetweenFactor<Pose3> > pose3Between = 
                boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
            Pose3 pose = pose3Between->measured();
            Key key1 = pose3Between->key1();
            Key key2 = pose3Between->key2();
            distMapper[i].addMeasure(Symbol(key1), Symbol(key2), pose);
            if(!distMapper[i].isCurrentRobot(Symbol(key1)))
            {
                for(size_t j = 0; j < 2; j++)
                {
                    if (distMapper[j].isCurrentRobot(Symbol(key1)))
                    {
                        Pose3 neighborPose = distMapper[j].getPoseAt(key1);
                        distMapper[i].addPose(Symbol(key1), neighborPose);
                    }
                }
            }
            if (!distMapper[i].isCurrentRobot(Symbol(key2)))
            {
                for(size_t j = 0; j < 2; j++)
                {
                    if (distMapper[j].isCurrentRobot(Symbol(key2)))
                    {
                        Pose3 neighborPose = distMapper[j].getPoseAt(key2);
                        distMapper[i].addPose(Symbol(key2), neighborPose);
                    }
                }
            }
        }
    }

    distMapper[0].separatorEdgeIds_.clear();
    distMapper[0].neighborRobotsPoses_.clear();
    distMapper[0].estimate().print();
    return 0;
}

int testTwoRobots()
{
    DistributedMapper distMapper[2];
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12);
    //
    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        auto initial = *(graphAndValue.second);
        auto name = Symbol(initial.keys()[0]).chr();
        distMapper[i].setRobotInfo(i, name);
    }

    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        
        noiseModel::Diagonal::shared_ptr rotModel = noiseModel::Isotropic::Variance(9, 1);
        noiseModel::Isotropic::shared_ptr posModel = noiseModel::Isotropic::Variance(12, 1);
        distMapper[i].setNoiseModel(posModel, rotModel);

        for (const Values::ConstKeyValuePair& key_value: *(graphAndValue.second))
        {
            Key key = key_value.key;
            Pose3 pose = key_value.value.cast<Pose3>();
            if(distMapper[i].isAddPrior())
            {
                distMapper[i].addPose(Symbol(key), pose);
            }
            else
            {
                distMapper[i].addPriorFactor(Symbol(key), pose, priorModel);
            }
        }
    }
    
    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        for (const boost::shared_ptr<NonlinearFactor>& factor: *(graphAndValue.first))
        {
            boost::shared_ptr<BetweenFactor<Pose3> > pose3Between = 
                boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
            Pose3 pose = pose3Between->measured();
            Key key1 = pose3Between->key1();
            Key key2 = pose3Between->key2();
            distMapper[i].addMeasure(Symbol(key1), Symbol(key2), pose);
        }
    }

    int maxIter = 1000;
    Values value[2];
    for (int n = 0; n < maxIter; n++)
    {
        for (int i = 0; i < 2; i++)
        {
            value[i] = distMapper[i].estimate();
        }
        // updata neighborhood poses
        // for (int i = 0; i < 2; i++)
        // {
        //     for (int j = 0; j < 2; j++)
        //     {
        //         if (i == j) continue;
        //         auto neighborKeys =  distMapper[i].getNeighborKeys(distMapper[j].getName());
        //         for (auto key: neighborKeys)
        //         {
        //             distMapper[i].updataNeighborPoses(Symbol(key),
        //                                               distMapper[j].getPoseAt(key));
        //         }
        //     }
        // }
    }
    value[0].print();
    value[1].print();
    return 0;
}

int centralized()
{
    CentralizedMapper centerMapper;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12);
    noiseModel::Diagonal::shared_ptr rotModel = noiseModel::Isotropic::Variance(9, 1);
    noiseModel::Isotropic::shared_ptr posModel = noiseModel::Isotropic::Variance(12, 1);
    centerMapper.setNoiseModel(posModel, rotModel);
    for (size_t i = 0; i < 2; i++)
    {
        std::string dataFile_i = "../data/" + boost::lexical_cast<std::string>(i) + ".g2o";
        auto graphAndValue = readG2o(dataFile_i, true);
        for (const Values::ConstKeyValuePair& key_value: *(graphAndValue.second))
        {
            Key key = key_value.key;
            Pose3 pose = key_value.value.cast<Pose3>();
            if(centerMapper.isAddPrior())
            {
                centerMapper.addPose(Symbol(key), pose);
            }
            else
            {
                centerMapper.addPriorFactor(Symbol(key), pose, priorModel);
            }
        }
        for (const boost::shared_ptr<NonlinearFactor>& factor: *(graphAndValue.first))
        {
            boost::shared_ptr<BetweenFactor<Pose3> > pose3Between = 
                boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
            Pose3 pose = pose3Between->measured();
            Key key1 = pose3Between->key1();
            Key key2 = pose3Between->key2();
            centerMapper.addMeasure(Symbol(key1), Symbol(key2), pose);
        }
    }
    centerMapper.estimate().print();//.print();
    
    return 0;
}
