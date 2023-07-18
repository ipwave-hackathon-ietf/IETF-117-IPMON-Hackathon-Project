#include <vector>
#include <map>
#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"


namespace inet{
struct node{
    std::string curNodeID;
    Coord NodePos;
    std::map<std::string, double> neighNodes;
    std::map<std::string, std::map<std::string, double>> flyingDrones; /*Nodes, Flying drones and their flight start time*/
};

class spacegraph: public LineSegmentsMobilityBase{
    private:
        std::vector<node> graphnodes;


    public:
        spacegraph();
        ~spacegraph();
        void buildgraphnodes(std::map<std::string, Coord> nodes);
        std::vector<node> returngraphnodes(){ return graphnodes;}
        void buildvolgraph(std::map<std::string, Coord> nds, double vx);
};
}
