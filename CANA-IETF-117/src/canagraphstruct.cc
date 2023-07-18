#include <math.h>
#include "canagraphstruct.h"

//#define DEBUG_GRAPHNODES

namespace inet{

canagraphstruct::canagraphstruct(){
}
canagraphstruct::~canagraphstruct(){

}
void canagraphstruct::buildgraphnodes1(std::map<std::string, Coord> nodes, Coord boundLim){
    std::cout <<"canagraphstruct.cc file!  Num Nodes = "<<nodes.size() <<"; Boundary = ("<<boundLim.x <<";"<<boundLim.y<<";"<<boundLim.z<<")"<<std::endl;
    int nx = boundLim.x + 1;
    int ny = boundLim.y + 1;
    int nz = boundLim.z + 1;
    double tx = 0;

    for (int i = 0; i < nx; i++){
        tx = i;
        double ty = 0;
        for (int j = 0; j < ny; j++){
            ty = j;
            double tz = 0;
            for (int k = 0; k < nz; k++){
                tz = k;
                vvnode tmpnode;
                Coord tmp, tmpx1, tmpx2, tmpy1, tmpy2, tmpz1, tmpz2;
                tmp.x = tx, tmp.y = ty, tmp.z = tz;
                tmpnode.np = tmp;
                if(i-1 > 0){
                    tmpx1.x = tmp.x-1, tmpx1.y = tmp.y, tmpx1.z = tmp.z;
                    tmpnode.prevadj1.push_back(tmpx1);
                }
                if(i+1 < nx){
                    tmpx2.x = tmp.x+1, tmpx2.y = tmp.y, tmpx2.z = tmp.z;
                    tmpnode.nextadj1.push_back(tmpx2);
                }
                if(j-1 > 0){
                    tmpy1.x = tmp.x, tmpy1.y = tmp.y-1, tmpy1.z = tmp.z;
                    tmpnode.prevadj1.push_back(tmpy1);
                }
                if(j+1 < ny){
                    tmpy2.x = tmp.x, tmpy2.y = tmp.y+1, tmpy2.z = tmp.z;
                    tmpnode.nextadj1.push_back(tmpy2);
                }
                if(k-1 > 0){
                    tmpz1.x = tmp.x, tmpz1.y = tmp.y, tmpz1.z = tmp.z-1;
                    tmpnode.prevadj1.push_back(tmpz1);
                }
                if(k+1 < nz){
                    tmpz2.x = tmp.x, tmpz2.y = tmp.y, tmpz2.z = tmp.z+1;
                    tmpnode.nextadj1.push_back(tmpz2);
                }
                volgraph1.push_back(tmpnode);
            }
        }
    }

    for (auto t1:nodes){
        double mindst = INFINITY;
        vvnode dstp;
        for (auto t2:volgraph1){
            double dst = sqrt(pow(t2.np.x-t1.second.x, 2) + pow(t2.np.y-t1.second.y, 2) + pow(t2.np.z-t1.second.z, 2));
            if (dst < mindst){
                mindst = dst;
                dstp = t2;
            }
        }
        dstnodes1.push_back(dstp);
    }

#ifdef DEBUG_GRAPHNODES
    for (auto i:volgraph1){
        std::cout <<"("<< i.np.x<<"; " <<i.np.y << "; " <<i.np.z <<"); Neighbors: " <<std::endl;
      /*  for(auto ii:i.neighNodes){
            std::cout <<ii.first <<" = " <<ii.second<<"  :  ";
        }*/
        std::cout <<std::endl;
    }
#endif
}
void canagraphstruct::buildgraphnodes(std::map<std::string, Coord> dst, std::map<std::string, Coord> bst,  Coord boundLim){
    std::cout <<"canagraphstruct.cc file!  Num Dst = "<<dst.size() <<"; Boundary = ("<<boundLim.x <<";"<<boundLim.y<<";"<<boundLim.z<<")"<<std::endl;
    double sidesize = 10;
    int nx = std::ceil(boundLim.x/sidesize);
    int ny = std::ceil(boundLim.y/sidesize);
    int nz = std::ceil(boundLim.z/sidesize);
    double tz = 0;
    std::vector<std::vector<std::vector<voxel>>> vox;
    int id = 0;

    for (int k = 0; k < nz; k++){
        tz = k;
        double ty = 0;
        std::vector<std::vector<voxel>> v2;
        for (int j = 0; j < ny; j++){
            ty = j;
            double tx = 0;
            std::vector<voxel> v1;
            for (int i = 0; i < nx; i++){
                voxel tmpnode;
                Coord tmp, tmpopp, cent;
                tx = i;
                tmp.x = tx*sidesize, tmp.y = ty*sidesize, tmp.z = tz*sidesize;
                tmpnode.id = id;
                tmpopp.x = tmp.x + sidesize, tmpopp.y = tmp.y + sidesize, tmpopp.z = tmp.z + sidesize;
                cent.x = (tmp.x + tmpopp.x)/2, cent.y = (tmp.y + tmpopp.y)/2, cent.z = (tmp.x + tmpopp.x)/2;
                tmpnode.cetroid=cent;
                v1.push_back(tmpnode);
            }
            v2.push_back(v1);
        }
        vox.push_back(v2);
    }
    for (int i = 0; i < vox.size(); i++){
        for (int ii = 0; ii < vox[i].size(); ii++){
            for (int iii = 0; iii < vox[i][ii].size(); iii++){
                int k1 = vox.size(), k2 = vox[i].size(), k3 = vox[i][ii].size();
                vnode np;
                np.cn = vox[i][ii][iii];

                if(i-1 >= 0){
                   np.prevadj.push_back(vox[i-1][ii][iii]);
                }
                if(i+1 < k1){
                    np.nextadj.push_back(vox[i+1][ii][iii]);
                }
                if(ii-1 >= 0){
                    np.prevadj.push_back(vox[i][ii-1][iii]);
                }
                if(ii+1 < k2){
                    np.nextadj.push_back(vox[i][ii+1][iii]);
                }
                if(iii-1 >= 0){
                    np.prevadj.push_back(vox[i][ii][iii-1]);
                }
                if(iii+1 < k3){
                    np.nextadj.push_back(vox[i][ii][iii+1]);
                }
                volgraph.push_back(np);
            }
        }
    }
    double minz = volgraph[0].cn.cetroid.z;
    for (auto t: dst){
        double mind = INFINITY;
        vnode dn;
        for (int i = 0; i < volgraph.size(); i++){
            if (volgraph[i].cn.cetroid.z == minz){
                double d = sqrt(pow(t.second.x-volgraph[i].cn.cetroid.x, 2) + pow(t.second.y-volgraph[i].cn.cetroid.y, 2) + pow(t.second.z-volgraph[i].cn.cetroid.z, 2));
                if (d < mind){
                    mind = d;
                    dn = volgraph[i];
                }
            }
        }
        dstnodes.push_back(dn);
    }
    for (auto t: bst){
          double mind = INFINITY;
          vnode bn;
          for (int i = 0; i < volgraph.size(); i++){
              if (volgraph[i].cn.cetroid.z == minz){
                  double d = sqrt(pow(t.second.x-volgraph[i].cn.cetroid.x, 2) + pow(t.second.y-volgraph[i].cn.cetroid.y, 2) + pow(t.second.z-volgraph[i].cn.cetroid.z, 2));
                  if (d < mind){
                      mind = d;
                      bn = volgraph[i];
                  }
              }
          }
          bstnodes.push_back(bn);
      }

#ifdef DEBUG_GRAPHNODES
    std::cout <<"<<<<< Centroids: ========================>>>>>"<<std::endl;
    for (auto i:volgraph){
        std::cout <<"("<< i.cn.cetroid.x<<"; " <<i.cn.cetroid.y << "; " <<i.cn.cetroid.z <<")" <<std::endl;
      /*  for(auto ii:i.neighNodes){
            std::cout <<ii.first <<" = " <<ii.second<<"  :  ";
        }*/
        std::cout <<std::endl;
    }
   std::cout <<"Destinations:: " <<std::endl;
    for (auto i:dstnodes){
        std::cout <<"("<< i.cn.cetroid.x<<"; " <<i.cn.cetroid.y << "; " <<i.cn.cetroid.z <<")" <<std::endl;
    }
    std::cout <<"Mission Bases: " <<std::endl;
    for (auto i:bstnodes){
        std::cout <<"("<< i.cn.cetroid.x<<"; " <<i.cn.cetroid.y << "; " <<i.cn.cetroid.z <<")" <<std::endl;
    }
#endif
}
void canagraphstruct::updateGraphNodes(std::vector<node> addupdate, std::vector<node> reductionupdate){
    for (auto i:reductionupdate){
        for (int j = 0; j < graphnodes.size(); j++){
            if (std::strcmp(graphnodes[j].curNodeID.c_str(), i.curNodeID.c_str()) == 0){
                for (auto k:i.flyingDrones){
                    std::map<std::string, double>::iterator it;
                    it = graphnodes[j].flyingDrones.find(k.first);
                    if (it != graphnodes[j].flyingDrones.end()){
                        graphnodes[j].flyingDrones.erase(k.first);
                    }
                }
                break;
            }
        }
    }
    for (auto i:addupdate){
        for (int j = 0; j < graphnodes.size(); j++){
            if (std::strcmp(graphnodes[j].curNodeID.c_str(), i.curNodeID.c_str()) == 0){
                for (auto k:i.flyingDrones){
                    graphnodes[j].flyingDrones.insert(std::make_pair(k.first,k.second));
                }
                break;
            }
        }
    }
}
}
