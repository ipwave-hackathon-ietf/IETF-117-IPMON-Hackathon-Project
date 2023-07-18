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

//    EV_TRACE << "initializing DroneMobility stage " << stage << endl;
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
        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
//            CubicConstruction ();
//            buildgraphnodes(CubicConstruction());
        }
/*        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            std::cout << " Name -----> " << getParentModule()->getFullName() << " DroneNetMob.cc" <<std::endl;
            if (numdst->intValue() > 0){
                destGen (numdst->intValue());
            }
            StatNodeMob* pt;
            std::map<std::string, Coord> src= pt->returnBStations();
//            std::cout << "SRC: " <<std::endl;
            allnodes = pt->returnDestinations();
            for (auto i:src){
                allnodes.insert(std::make_pair(i.first, i.second));
            }
//            buildgraphnodes(allnodes);

            parcelsDefinition(nparcels);
           for (auto i:parcel_depot){
               std::cout << i.parcelID <<" ; W =" <<i.weight <<"; P = " << i.priority << "; Exp = " <<i.exp_time
                       <<" Pos = (" <<i.parceldest.x <<"; " << i.parceldest.y <<"; " <<i.parceldest.z <<")" << std::endl;
           }
        }*/
//        targetPosition = missionPathNextDest_n(mBasePos.NodePos);
       /* std::cout << " Name -----> " << getParentModule()->getFullName() <<
                    " And Src = (" << mBasePos.NodePos.x <<"; "<<mBasePos.NodePos.y <<"; " <<mBasePos.NodePos.z <<")" << std::endl;*/
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
 //            std::cout << "SRC: " <<std::endl;
             allnodes = pt->returnDestinations();
             for (auto i:src){
                 allnodes.insert(std::make_pair(i.first, i.second));
             }
 //            buildgraphnodes(allnodes);
             parcelsDefinition(nparcels);
 /*           for (auto i:parcel_depot){
                std::cout << i.parcelID <<" ; W =" <<i.weight <<"; P = " << i.priority << "; Exp = " <<i.exp_time
                        <<" Pos = (" <<i.parceldest.x <<"; " << i.parceldest.y <<"; " <<i.parceldest.z <<")" << std::endl;
            }*/
         }
        targetPosition = missionPathNextDest_n(mBasePos.NodePos);
        isdestinationset = true;
    }
    if (flag_original){  /*Original setTargetPosition definition from INET mass mobility*/
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
        /*Modified setTargetPosition function for drone mobility
         * Destination is defined according to the selection algorithm chosen*/
        simtime_t nextChangeInterval = *changeIntervalParameter;
        sourcePosition = lastPosition;
//        targetPosition = missionPathNextDest(lastPosition);
        targetPosition = missionPathNextDest_n(lastPosition);

        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
}

void DroneNetMob::move()
{
//    std::cout<<"======= DroneNetMob::move() ==========" <<std::endl;

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
//            std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);

          /*  if (std::strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
                std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
            }*/

/*           std::cout << " Name -----> " << getParentModule()->getFullName() << " , Velocity = ("
                            << getCurrentVelocity().x << "; " << getCurrentVelocity().y << "; "<< getCurrentVelocity().z <<"), Position = ("
                            << getCurrentPosition().x<< "; " << getCurrentPosition().y <<"; "<< getCurrentPosition().z <<")"<< std::endl;*/
        }
    }
}

double DroneNetMob::getMaxSpeed() const
{
//    std::cout<<"DroneNetMob::getMaxSpeed()" <<std::endl;
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}
/*This function enqueues the destinations in a way that drones can access while defining their path
 * */
void DroneNetMob::destGen(int ndst){
    bool flagprev = false;
    /*Randomly define the destinations
     * */
    if (flagprev){
        for (unsigned int i = 0; i < numdst->intValue(); i++){
            Coord nextdst;
            nextdst.x = rand() % 600;
            nextdst.y = rand() % 400;
            nextdst.z = 0;
            dst.push_back(nextdst);
        }
    }
    /*Through a pointer access the destinations
        * Enqueue  their positions for drone trajectories */
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
/*    std::cout << " Selection ===>  " << parcelSel << std::endl;
    std::cout << " Drone -----> " << getParentModule()->getFullName() <<
                    " with speed = " << speedParameter->doubleValue() <<"Defines its mission!"<< std::endl;*/
    switch(parcelSel){
        /* Closest-Deadline-Parcel-First
         * Depot sorted by Deadline
         * First deliver the ones with small deadline*/
        case CDPF:{
/*            std::cout <<" CDPF ----- >    Parcel Selection Method! parcel_depot size = " <<parcel_depot.size() << std::endl;*/
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);
/*                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    std::cout <<"P" <<parcel_depot[i].parcelID << ":: " << "W = " <<parcel_depot[i].weight <<" dst = (" <<parcel_depot[i].parceldest.x <<"; "<<parcel_depot[i].parceldest.y <<"; "
                            <<parcel_depot[i].parceldest.z <<")"<<" Deadline = "<< parcel_depot[i].exp_time  <<std::endl;
                }*/
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
/*    for (auto i:selectedParcels){
        std::cout <<"+++ ID = " << i.parcelID << " Weight: " <<i.weight <<" deadline = " << i.exp_time <<
                "   <<-->> parcel_depot Size = "<< parcel_depot.size() <<std::endl;
    }*/
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
//            nextdest = originPos;
//            std::cout << "::  originPos  ::  ("<<originPos.x <<"; " <<originPos.y <<"; " <<originPos.z <<")" <<std::endl;
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
   /* std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
            <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;*/
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
//            std::cout << "::  mBasePos  ::  ("<<mBasePos.x <<"; " <<mBasePos.y <<"; " <<mBasePos.z <<")" <<std::endl;
//            nextdest.x = 0, nextdest.y = 0, nextdest.z = 0;
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
   /* std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
            <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;*/
    dest.x = nextdest.x + Xerr;
    dest.y = nextdest.y + Yerr;
    dest.z = nextdest.z;
    return dest;
}

/*Algorithm for destination selection based on:
 * Shortest path and drone capacity
 * */
Coord DroneNetMob::destAssignment(){
    int sz = dst.size();
    Coord ds = dst[gen];
    gen++;
    return ds;
}
/*Multiple destination visit (2-4) that are randomly chosen amongst the destinations
 * visit them following the shortest path
 * */
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
/*Evaluate the congestion cost at the planned next destination
 * Estimate the cost of moving to alternative destination compared to the planned next destination
 * Decide drone's next movement.*/
node DroneNetMob::selectNextFlightDst(node CurDst){
    node NxtDst;
    std::vector<node> GrN = returngraphnodes();
    node planNextNode = MissionPlanedNodes.front();
    double congCost = 0;
    double DelayCost = 0;

    //COME BACK ON IT
 /*   for (auto i:GrN){
        if (std::strcmp(i.curNodeID.c_str(), planNextNode.curNodeID.c_str()) == 0){
            int congSize = i.flyingDrones.size();
            congCost = congSize*2.5;//Assuming that it takes 2.5 seconds to serve a drone at the destination.
        }
        if (std::strcmp(i.curNodeID.c_str(), CurDst.curNodeID.c_str()) == 0){
            for(auto ii : i.neighNodes){
                if (std::strcmp(ii.first.c_str(), planNextNode.curNodeID.c_str())){
                    DelayCost = ii.second;
                }
            }
        }
    }*/
    double NextDestCost = congCost + DelayCost;
    NxtDst = planNextNode;

    return NxtDst;
}
std::vector<std::vector<std::vector<std::map<int,Coord>>>> DroneNetMob::CubicConstruction (){
    double Pxlength = 500;
    double Pylength = 500;
    double Pzlength = 200;
    double voxside  = 1;
    int ni = (int) Pxlength/voxside;
    int nj = (int) Pylength/voxside;
    int nk = (int) Pzlength/voxside;
    std::cout <<"NPoints = "<<"("<<ni<<"; " <<nj <<"; "<< nk <<") ";
    std::vector<std::vector<std::vector<std::map<int,Coord>>>> Cb;
    std::vector<std::vector<std::vector<std::map<int,Coord>>>>::iterator it1;
    std::vector<std::vector<std::map<int,Coord>>>::iterator it2;
    std::vector<std::map<int,Coord>>::iterator it3;

    double tmpx = 0;
    Coord orig;
    orig.x = 0, orig.y =0, orig.z = 0;
    int id = 1;
    for (int i = 0; i < ni; i++){
        double px, py, pz;
        Coord C; // voxel center
        pz = i*voxside;
        std::vector<std::vector<std::map<int, Coord>>> Pts;
        for (int j = 0; j < nj; j++){
            py = j*voxside;
            std::vector<std::map<int, Coord>> P;
            for (int k = 0; k < nk; k++){
                px = k*voxside;
                std::map<int, Coord> tmpnd;
                C.x =orig.x + ((px-orig.x)/2), C.y =orig.y + ((py-orig.y)/2), C.z =orig.z + ((pz-orig.z)/2);
//                std::cout <<" ("<< C.x <<"; "<< C.y<<"; "<< C.z <<") : ";
                tmpnd.insert(std::make_pair(id, C));
                P.push_back(tmpnd);
                orig.x=C.x;
            }
            Pts.push_back(P);
            orig.y = C.y;
            P.clear();
        }
        orig.z = C.z;
        Cb.push_back(Pts);
        Pts.clear();
    }
    std::cout <<"S1 = " <<Cb.size() << "; S2 = " <<Cb[0].size() <<"; S3 = " <<Cb[0][0].size() <<std::endl;
#ifdef voxdebug


    for (it1 = Cb.begin(); it1 != Cb.end(); it1++){
        for (it2 = (*it1).begin(); it2 != (*it1).end(); it2++){
            for (it3 = (*it2).begin(); it3 != (*it2).end(); it3++){
                for (auto i:*it3)
                std::cout <<"("<<i.second.x<<"; " <<i.second.y <<"; "<< i.second.z <<") ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }
#endif
    return Cb;
}
} // namespace inet
