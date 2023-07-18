/*
 * DroneScenario.h
 *
 *  Created on: Jun 24, 2021
 *      Author: iotlab_aime
 */


#include "inet/common/INETDefs.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <list>
#include <queue>
#include "FusionEKF.h"


namespace inet{

class DroneNetMob;

class DroneScenario: public cSimpleModule{
    public:
    DroneScenario();
    ~DroneScenario();
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
    virtual void handleSelfMsg(cMessage *msg);
    double penetrationrate;
    int MissionID;
    DroneNetMob* getDroneNetMob() const { return commandIfc; }
    std::map<std::string, cModule*> rerurnFlyingDrones(){ return hosts;}


    protected:
    simtime_t firstStepAt; /**< when to start injections */
    simtime_t updateInterval; /**< time interval of hosts' position updates */
    std::set<std::string> queuedDrones;
    std::map<std::string, cModule*> hosts; /**< vector of all hosts managed by us */
    std::string Host;
    std::string ModuleName;
    int numdrones;
    int numberofPenetraDrones;
    cRNG* mobRng;
    double penetrationRate; /*Drone arrival rate Lambda*/
    simtime_t penetrationTime;
    uint32_t vehicleNameCounter;
    int vehicleRngIndex;
    int numDrones;
    cMessage* executeOneTimestepTrigger; /**< self-message scheduled for when to next call executeOneTimestep */
    uint32_t activeDroneCount; /**< n**/
    DroneNetMob* commandIfc;
    size_t nextNodeVectorIndex; /**< next OMNeT++ module vector index to use */
    void simpleModuleAdd();
};

}

