#include <vector>
#include <map>
#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"


namespace inet{

struct node{
    std::string curNodeID;
    Coord NodePos;
    std::map<std::string, double> neighNodes;
    std::map<std::string, double> flyingDrones; /*Flying drones and their flight start time*/
};
struct voxel{
    int id;
    Coord cetroid;
};
struct vvnode{
    Coord np;
    std::vector<Coord> nextadj1;
    std::vector<Coord> prevadj1;
};

struct vnode{
    voxel cn;
    std::vector<voxel> nextadj;
    std::vector<voxel> prevadj;
};
class canagraphstruct: public LineSegmentsMobilityBase{
    private:
        std::vector<node> graphnodes;
        std::vector<vvnode> volgraph1;
        std::vector<vvnode> dstnodes1;
        std::vector<vnode> volgraph;
        std::vector<vnode> dstnodes;
        std::vector<vnode> bstnodes;


    public:
        canagraphstruct();
        ~canagraphstruct();
        void buildgraphnodes1(std::map<std::string, Coord> nodes, Coord boundLim);
        void buildgraphnodes(std::map<std::string, Coord> dst, std::map<std::string, Coord> bst, Coord boundLim);
        std::vector<vnode> returngraphnodes(){ return volgraph;}
        std::vector<vnode> returndests(){ return dstnodes;}
        std::vector<vnode> returnbsts(){ return bstnodes;}
        void updateGraphNodes(std::vector<node> addupdate, std::vector<node> reductionupdate);

};
}
