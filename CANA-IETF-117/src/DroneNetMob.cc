/*
 * DroneNetMob.cc
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */


#include "inet/common/INETMath.h"
#include "DroneNetMob.h"
#include "stationaryNodeMob.h"
#include <fstream>
#include "tools.h"
#include "inet/common/INETMath.h"


namespace inet {

bool flag_original = false;
Coord originPos;
std::vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 5000;
bool flagArrangedDepot = false;
bool OngoingMission = false;
std::map<std::string, Coord> allnodes;

std::vector<parcel> parcel_depot;

Define_Module(DroneNetMob);
bool sortDepotByDeadline (parcel i, parcel j) {
    return (i.exp_time < j.exp_time);
}
bool sortDepotByDestination (parcel i, parcel j) {
    return ((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
                 +pow(i.parceldest.z - originPos.z, 2))) < (sqrt(pow(j.parceldest.x - originPos.x, 2)
                    + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2))));
}
bool greedySortDepot (parcel i, parcel j) {
    return (((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
            +pow(i.parceldest.z - originPos.z, 2)))/i.weight) < ((sqrt(pow(j.parceldest.x - originPos.x, 2)
               + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2)))/j.weight));
}
bool SortDepotByWeight (parcel i, parcel j) {
    return (i.weight > j.weight);
}
DroneNetMob::DroneNetMob()
{
}

void DroneNetMob::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        rad heading = deg(par("initialMovementHeading"));
        rad elevation = deg(par("initialMovementElevation"));
        changeIntervalParameter = &par("changeInterval");
        angleDeltaParameter = &par("angleDelta");
        rotationAxisAngleParameter = &par("rotationAxisAngle");
        speedParameter = &par("speed");
        quaternion = Quaternion(EulerAngles(heading, -elevation, rad(0)));
        WATCH(quaternion);
        numdst = &par("ndst");
        ox =&par("initialX");
        oy = &par("initialY");
        oz = &par("initialZ");
        mBasePos.NodePos.x = ox->doubleValue();
        mBasePos.NodePos.y = oy->doubleValue();
        mBasePos.NodePos.z = oz->doubleValue();
        originPos = mBasePos.NodePos;

        droneweightcapacity =  (&par("weightCapacity"))->doubleValue();
        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
        selectionMethod = (&par("parcelSelecitionMethod"))->intValue();
    }
    else if (stage == INITSTAGE_LAST){
        std::cout << " Name -----> " << getParentModule()->getFullName() << " DroneNetMob.cc" <<std::endl;

    }
}

void DroneNetMob::orient()
{
    if (faceForward)
        lastOrientation = quaternion;
}

void DroneNetMob::setTargetPosition()
{
    if (!isdestinationset){
        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
             std::cout << " Name -----> " << getParentModule()->getFullName() << " DroneNetMob.cc" <<std::endl;
             if (numdst->intValue() > 0){
                 destGen (numdst->intValue());
             }
             stationaryNodeMob* pt;
             std::map<std::string, Coord> src= pt->returnBStations();
             allnodes = pt->returnDestinations();
             for (auto i:src){
                 allnodes.insert(std::make_pair(i.first, i.second));
             }
             parcelsDefinition(nparcels);
         }
        targetPosition = missionPathNextDest_n(mBasePos.NodePos);
        isdestinationset = true;
    }
    if (flag_original){
        rad angleDelta = deg(angleDeltaParameter->doubleValue());
        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
        quaternion = quaternion * dQ;
        quaternion.normalize();
        Coord direction = quaternion.rotate(Coord::X_AXIS);

        simtime_t nextChangeInterval = *changeIntervalParameter;
        EV_DEBUG << "interval: " << nextChangeInterval << endl;
        sourcePosition = lastPosition;
        targetPosition = lastPosition + direction * (*speedParameter) * nextChangeInterval.dbl();
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
    else{
        simtime_t nextChangeInterval = *changeIntervalParameter;
        sourcePosition = lastPosition;
        targetPosition = missionPathNextDest_n(lastPosition);

        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
}

void DroneNetMob::move()
{
    if (flag_original){ /*Original move definition from INET mass mobility*/
        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
    }
    else{ /*Modified move function for drone mobility*/
        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
    }
}

double DroneNetMob::getMaxSpeed() const
{
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}

void DroneNetMob::destGen(int ndst){
    bool flagprev = false;
    if (flagprev){
        for (unsigned int i = 0; i < numdst->intValue(); i++){
            Coord nextdst;
            nextdst.x = rand() % 600;
            nextdst.y = rand() % 400;
            nextdst.z = 0;
            dst.push_back(nextdst);
        }
    }
    else{

        stationaryNodeMob* pt;
        std::map<std::string, Coord> dts;
        dts = pt->returnDestinations();
        std::cout << "DST: " <<std::endl;
        for (auto i:dts){
            dst.push_back(i.second);
            allnodes.insert(std::make_pair(i.first, i.second));
        }
        std::cout <<" Assigned destinations:" <<std::endl;
        for (auto i:dst){
            std::cout <<"(" <<i.x <<"; " <<i.y <<"; " <<i.z <<") :: ";
        }
    }
}
void DroneNetMob::parcelsDefinition (int nparcels){
    for (unsigned int i = 0; i < nparcels; i++){
        parcel tmpparcel;
        parcel *p;
        tmpparcel.parcelID = i;
        tmpparcel.weight =  rand() % 10 + 1;
        tmpparcel.priority = 1;
        tmpparcel.exp_time = rand() % 300;
//        int n = numdst->intValue();
        int n = dst.size();
        int dindex = rand() % n;
        tmpparcel.parceldest = dst[dindex];
        parcel_depot.push_back(tmpparcel);

    }
}
std::vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    std::vector<parcel> selectedParcels;
    double packedweight = 0;
    switch(parcelSel){
        case CDPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Closest-Neighbor-Parcel-First
         * Depot sorted by Positions
         * First deliver the ones closer to source*/
        case CNPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Efficient Parcel Delivery Service
         * Depot sorted in a greedy way ()
         * First deliver the ones with small ratio distance/weight*/
        case EPDS:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Randomly-Selected-Parcel-First
         * Randomly select Parcels to be delivered first*/
        case RSPF:{
            if (!flagArrangedDepot){

            }
            else{

            }
            break;
        }
        /* Heaviest Parcel First
         * Depot is sorted based on parcel weight
         * First deliver the heaviest ones to the lightest*/
        case HPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),SortDepotByWeight);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        default:{
            std::cout <<" Undefined Selection Method." <<std::endl;
        }
    }
    return selectedParcels;
}
Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest, dest;
    double Xerr = -5 + rand() % 10 + 1;
    double Yerr = -5 + rand() % 10 + 1;
    if (!OngoingMission){
        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);

        double nearestDist = 0;
        int k = 0; //Next Parcel Index
        for (unsigned int i = 0; i < MissionParcels.size(); i++){
            if (i == 0){
                double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                    + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                    +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                nextdest = MissionParcels[i].parceldest;
                nearestDist = tmpd;
                k = i;
            }
            else{
                double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                    + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                    +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                if (tmpd < nearestDist){
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
            }
        }
        MissionParcels.erase(MissionParcels.begin()+k);


        OngoingMission = true;
    }
    else{
        if (MissionParcels.size() == 0){
            nextdest.x = 0, nextdest.y = 0, nextdest.z = 0;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
            for (unsigned int i = 0; i < MissionParcels.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
                else{
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionParcels[i].parceldest;
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionParcels.erase(MissionParcels.begin()+k);
        }
    }
    dest.x = nextdest.x + Xerr;
    dest.y = nextdest.y + Yerr;
    dest.z = nextdest.z;
    return dest;
}

Coord DroneNetMob::missionPathNextDest_n(Coord cpos){
    Coord nextdest, dest;
    double Xerr = -5 + rand() % 10 + 1;
    double Yerr = -5 + rand() % 10 + 1;
    if (!OngoingMission){
        MissionPath  = droneMissionDestinationSelection();

        double nearestDist = 0;
        int k = 0; //Next Parcel Index
        for (unsigned int i = 0; i < MissionPath.size(); i++){
            if (i == 0){
                double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                    + pow(MissionPath[i].y - cpos.y, 2)
                                    +pow(MissionPath[i].z - cpos.z, 2));
                nextdest = MissionPath[i];
                nearestDist = tmpd;
                k = i;
            }
            else{
                double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                    + pow(MissionPath[i].y - cpos.y, 2)
                                    +pow(MissionPath[i].z - cpos.z, 2));
                if (tmpd < nearestDist){
                    nextdest = MissionPath[i];
                    nearestDist = tmpd;
                    k = i;
                }
            }
        }
        MissionPath.erase(MissionPath.begin()+k);

        OngoingMission = true;
    }
    else{
        if (MissionPath.size() == 0){
            nextdest = mBasePos.NodePos;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
            for (unsigned int i = 0; i < MissionPath.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                        + pow(MissionPath[i].y - cpos.y, 2)
                                        +pow(MissionPath[i].z - cpos.z, 2));
                    nextdest = MissionPath[i];
                    nearestDist = tmpd;
                    k = i;
                }
                else{
                    double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                        + pow(MissionPath[i].y - cpos.y, 2)
                                        +pow(MissionPath[i].z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionPath[i];
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionPath.erase(MissionPath.begin()+k);
        }
    }
    dest.x = nextdest.x + Xerr;
    dest.y = nextdest.y + Yerr;
    dest.z = nextdest.z;
    return dest;
}
Coord DroneNetMob::destAssignment(){
    int sz = dst.size();
    Coord ds = dst[gen];
    gen++;
    return ds;
}
std::vector<Coord> DroneNetMob::droneMissionDestinationSelection(){
    int numdst;
    std::vector<Coord> seldst;
    std::vector<int> ind;
    int n = rand() % (4 - 2 + 1) + 2;
    for (int i = 0; i < n; i++){
        int k = rand() % 6;
        ind.push_back(k);
        seldst.push_back(dst[k]);
    }
    return seldst;
}
node DroneNetMob::selectNextFlightDst(node CurDst){
    node NxtDst;
    std::vector<node> GrN = returngraphnodes();
    node planNextNode = MissionPlanedNodes.front();
    double congCost = 0;
    double DelayCost = 0;
    double NextDestCost = congCost + DelayCost;
    NxtDst = planNextNode;

    return NxtDst;
}
} // namespace inet
