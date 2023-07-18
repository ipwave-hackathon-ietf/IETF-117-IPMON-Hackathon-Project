#include "inet/common/INETDefs.h"
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <map>

namespace inet{
using namespace std;
using namespace omnetpp;
class Destination : public cSimpleModule{
    private:
        virtual void initialize();
        virtual void handleMessage(cMessage *msg);
        cMessage* droneArrival;

    public:
        Destination();
        ~Destination();

};
}
