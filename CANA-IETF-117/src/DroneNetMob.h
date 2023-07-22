/*
 * DroneNetMob.h
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#ifndef __INET_DRONENETMOB_H
#define __INET_DRONENETMOB_H
#include <vector>
#include <algorithm>
#include <iostream>
#include "graphstruct.h"
#include <cmath>


namespace inet {

/**
 * @brief Random mobility model for a mobile host with a mass.
 * See NED file for more info.
 *
 * @author Emin Ilker Cetinbas, Andras Varga
 */
struct parcel{
    int parcelID;
    double weight;
    int priority;
    double exp_time;
    Coord parceldest;
};

void parcelsDefinition (int nparcels);

enum parcelSelection{
    CDPF = 0,        //Closest-Deadline-Parcel-First
    CNPF = 1,       //Closest-Neighbor-Parcel-First
    EPDS = 2,      //Efficient Parcel Delivery Service distance/weight
    RSPF = 3,     //Randomly-Selected-Parcel-First
    HPF  = 4     //Heaviest Parcel First
};

class INET_API DroneNetMob : public spacegraph
{
  protected:
    // config (see NED file for explanation)
    cPar *changeIntervalParameter = nullptr;
    cPar *angleDeltaParameter = nullptr;
    cPar *rotationAxisAngleParameter = nullptr;
    cPar *speedParameter = nullptr;
    cPar *numdst = nullptr;
    cPar *ox = nullptr;
    cPar *oy = nullptr;
    cPar *oz = nullptr;

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
//    bool flagmovedtodst;
    double droneweightcapacity;
    double droneremainingbattery;
    int selectionMethod;
    std::vector<parcel> MissionParcels;
    std::vector<Coord> MissionPath;

    double deliveryStartTime = 0;
    double deliveryEndTime =   0;
    bool isdestinationset = false;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Move the host according to the current simulation time. */
    virtual void move() override;
    void orient() override;

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition() override;



  public:
    DroneNetMob();
    virtual double getMaxSpeed() const override;
    void destGen(int ndst);
    void parcelsDefinition (int nparcels);
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord missionPathNextDest_n(Coord cp);
    Coord destAssignment();
    void managethedestcongestions();
    std::vector<Coord> droneMissionDestinationSelection();
    void shortestPathTraversal();
    node mBasePos; /*Mission base node*/
    std::vector<node> MissionPlanedNodes;/*Nodes to fly for the current Mission*/
    node curNode; /*/The node the drone is flying towards;*/
    node selectNextFlightDst(node curdst); /*return the next Node to fly towards*/
    void destinationHandle(); /*Handle self arrival to the destination based on all other arrivals*/
    std::vector<std::vector<std::vector<std::map<int,Coord>>>> CubicConstruction ();

};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H
