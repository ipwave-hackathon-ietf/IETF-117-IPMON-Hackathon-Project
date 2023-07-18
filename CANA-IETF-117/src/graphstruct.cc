#include "graphstruct.h"
#include <math.h>

//#define DEBUG_GRAPHNODES

namespace inet{

spacegraph::spacegraph(){
}
spacegraph::~spacegraph(){

}
void spacegraph::buildgraphnodes(std::map<std::string, Coord> nodes){
    std::cout <<"Nodes = " <<nodes.size()<<std::endl;
    for (std::map <std::string, Coord>::iterator i = nodes.begin(); i != nodes.end(); i++ ){
        node tmpn;
        tmpn.curNodeID = i->first;
        tmpn.NodePos = i->second;
        for (std::map <std::string, Coord>::iterator ii = nodes.begin(); ii != nodes.end(); ii++){
            if (std::strcmp(i->first.c_str(), ii->first.c_str()) != 0){
                double dstnce =sqrt(pow((i->second.x - ii->second.x), 2) + pow((i->second.y - ii->second.y), 2) + pow((i->second.z - ii->second.z), 2));
                tmpn.neighNodes.insert(std::make_pair(ii->first, dstnce));
            }
        }
        graphnodes.push_back(tmpn);
    }

#ifdef DEBUG_GRAPHNODES
    for (auto i:graphnodes){
        std::cout <<i.curNodeID <<"; ("<< i.NodePos.x<<"; " <<i.NodePos.y << "; " <<i.NodePos.z <<"); Neighbors: " <<std::endl;
        for(auto ii:i.neighNodes){
            std::cout <<ii.first <<" = " <<ii.second<<"  :  ";
        }
        std::cout <<std::endl;
    }
#endif
}
}
