#include "inet/common/INETMath.h"
#include "Destination.h"

namespace inet{

Define_Module(Destination);

Destination::Destination(){

}
Destination::~Destination(){

}
void Destination::initialize(){
    droneArrival = new cMessage("Arrival of Drone");
//    std::cout <<" <  ======  > Destination, Name = " << getParentModule()->getFullName() <<std::endl;
}
void Destination::handleMessage(cMessage* msg){
    if (msg == droneArrival){

    }
    else{

    }
}

}

