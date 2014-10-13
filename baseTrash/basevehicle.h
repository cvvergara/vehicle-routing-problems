#ifndef BASEVEHICLE_H
#define BASEVEHICLE_H

#include <limits>
#include <vector>
#include <sstream>


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "plot.h"
#include "move.h"


class BaseVehicle  {
  protected:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID ;
    inline double _MAX() const { ( std::numeric_limits<double>::max() ); };
    inline double _MIN() const { ( - std::numeric_limits<double>::max() ); };


    int vid;
    int ntype;
    Twpath<Trashnode> path;
    Trashnode depot; //just for keeps
    Trashnode endingSite;
    Trashnode dumpSite;

    double maxcapacity;
    double cost;        // cost of the route

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  protected:

    // this is used when we save a copy of the path so we can make
    // changes and to restore the original path if the changes
    // do not improve the path.
    // There is a hidden assumption that path[0] == endingSite node.

    void setvpath(Twpath<Trashnode> p) { path = p; };

  public:

    bool isvalid() const {return vid>=0;};  //
    bool findNearestNodeTo(Bucket &unassigned,const TWC<Trashnode> &twc,  UID &pos,  Trashnode &bestNode);
    double getCurrentCapacity() const {return maxcapacity - path[size()-1].getcargo();}
    bool e_setPath(const Bucket &sol);
    //--------------------------------------------------------------------
    // structors
    //--------------------------------------------------------------------

    BaseVehicle() {
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

    BaseVehicle(std::string line,const Bucket &otherlocs)  {
       // TESTED on running program
       assert(otherlocs.size());
       std::istringstream buffer( line );
       int depotId,depotNid;
       int dumpId,dumpNid;
       int endingId,endingNid;
       double dumpServiceTime;
       double endTime,startTime;
       endTime=startTime=0;

       cost        = 0;
       w1 = w2 = w3 = 1.0;

       buffer >> vid;
       buffer >> depotId;
       buffer >> dumpId;
       buffer >> endingId;
       buffer >> dumpServiceTime;
       buffer >> maxcapacity;
       buffer >> startTime;
       buffer >> endTime;

       if (depotId>=0 and dumpId>=0 and endingId>=0 and startTime>=0 and startTime<=endTime and maxcapacity>0 and dumpServiceTime>=0 and vid>=0
	  and otherlocs.hasId(depotId) and otherlocs.hasId(dumpId) and otherlocs.hasId(endingId)){ 

          endingSite=otherlocs[otherlocs.posFromId(endingId)];
	  if (endingSite.closes() > endTime) endingSite.setCloses(endTime);
          dumpSite=otherlocs[otherlocs.posFromId(dumpId)];
          dumpSite.setServiceTime(dumpServiceTime);
          depot=otherlocs[otherlocs.posFromId(depotId)];
	  if (depot.opens() < startTime) depot.setOpens(startTime);

	  depot.setType(0);
	  depot.setDemand(0);
	  dumpSite.setType(1);
	  endingSite.setType(3);
          push_back(depot);
          evalLast();
dumpeval();

       } else vid=-1;  //truck is rejected
   }


    //--------------------------------------------------------------------
    // accessors
    //--------------------------------------------------------------------

    Twpath<Trashnode> getvpath() const { return path; };
    Twpath<Trashnode>& getvpath() { return path; };
    std::deque<int> getpath() const;
    int size() const { return path.size(); };
    int getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return endingSite.gettwvTot(); };
    int getCV() const { return endingSite.getcvTot(); };
    int getcargo() const { return  path[path.size()-1].getcargo(); };
    double getduration() const { return (path.size()-1 == 0)?0.0:endingSite.getTotTime(); };
    double getcost() const { return cost; };
    double getTimeOSRM() const;
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };
    int getVid() const { return vid; };
    Trashnode getdepot() const { return endingSite; };
    Trashnode& getdepot() { return endingSite; };
    Trashnode getdumpSite() const { return dumpSite; };
    const Trashnode& getBackToDepot() const {return endingSite;}
    const Trashnode& getEndingSite() const {return endingSite;}
    const Trashnode& getStartingSite() const {return depot;}
    const Trashnode& getdumpSite() { return dumpSite; };

    double distancetodepot(int i) const { return path[i].distance(getdepot()); };
    double distancetodump(int i) const { return path[i].distance(getdumpSite()); };

    const Trashnode& operator[](int i) const { return path[i]; };
    //Trashnode operator[](int i) const { return path[i]; };

    //--------------------------------------------------------------------
    // dumps and plots
    //--------------------------------------------------------------------
    void dump() const;
    void dump(const std::string &title) const;
    void dumpeval() const;
    void smalldump() const;
    void dumppath() const;
    void tau() const ;

    void plot(std::string file,std::string title,int carnumber);
    void plot(Plot<Trashnode> graph,int carnumber);


    //--------------------------------------------------------------------
    // evaluation
    //--------------------------------------------------------------------

    bool feasable() const { return endingSite.gettwvTot() == 0 and endingSite.getcvTot() == 0; };
    bool hascv()const { return endingSite.getcvTot() != 0; };
    bool hastwv()const { return endingSite.gettwvTot() != 0; };

    void evalLast();

    //--------------------------------------------------------------------
    // mutators 
    //--------------------------------------------------------------------

    // these two do not work with autoeval
    // instead use BaseVehicle(depot, dump) constructor
    //void setdepot(Trashnode _depot) { endingSite = _depot; };
    //void setdumpSite(Trashnode _dump) { dumpSite = _dump; };

    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    //--------------------------------------------------------------------
    // wrappers to twpath code to 
    //--------------------------------------------------------------------

    // single path manipulators

    bool push_back(Trashnode node);
    bool push_front(Trashnode node);
    bool insert(Trashnode node, int at);
    bool remove( int at);
    bool moverange( int rangefrom, int rangeto, int destbefore );
    bool movereverse( int rangefrom, int rangeto, int destbefore );
    bool reverse( int rangefrom, int rangeto );
    bool move( int fromi, int toj );
    bool swap( const int& i, const int& j );

    // multiple path manipulators

    bool swap(BaseVehicle& v2, const int& i1, const int& i2);

    // restore a saved path to undo an operation

    void restorePath(Twpath<Trashnode> oldpath); //tmp=v; v.dostuff; v=tmp restores as original

    //--------------------------------------------------------------------
    // algorithm specific - intra-route manipulations
    //--------------------------------------------------------------------

    bool doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4);
    bool doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6);
    bool doOrOpt(const int& c1, const int& c2, const int &c3);
    bool doNodeMove(const int& i, const int& j);
    bool doNodeSwap(const int& i, const int& j);
    bool doInvertSeq(const int& i, const int& j);

    bool pathOptimize();
    bool pathTwoOpt();
    bool pathThreeOpt();
    bool pathOrOpt();
    bool pathOptMoveNodes();
    bool pathOptExchangeNodes();
    bool pathOptInvertSequence();

    bool findBestFit(const Trashnode &node, int* tpos, double* deltacost);

    //--------------------------------------------------------------------
    // algorithm specific - inter-route manipulations
    //--------------------------------------------------------------------

    bool swap2(BaseVehicle& v2, const int& i1, const int& i2, bool force);
    bool swap3(BaseVehicle& v2, BaseVehicle& v3, const int& i1, const int& i2, const int& i3, bool force);
    bool exchangeSeq(BaseVehicle& v2, const int& i1, const int& j1, const int& i2, const int& j2, bool force);
    bool exchangeTails(BaseVehicle& v2, const int& i1, const int& i2, bool force);
    bool exchange3(BaseVehicle& v2, BaseVehicle& v3, const int& cnt, const int& i1, const int& i2, const int& i3, bool force);
    bool relocate(BaseVehicle& v2, const int& i1, const int& i2, bool force);
    bool relocateBest(BaseVehicle& v2, const int& i1);




    //----------------------------------------------------------------
    // I really hate these shortcuts & I love them but I'll think about them really hard
    //----------------------------------------------------------------

    Bucket  Path() const { return path; };
    int getnid(int i) const { return path[i].getnid(); };
    int getid(int i) const { return path[i].getid(); };
    double getx(const int i) const { path[i].getx(); };
    double gety(const int i) const { path[i].gety(); };
    bool hasdemand(int i) const { return path[i].hasdemand(); };
    bool hassupply(int i) const { return path[i].hassupply(); };
    bool hasnogoods(int i) const { return path[i].hasnogoods(); };
    bool earlyarrival(int i, const double D) const { return path[i].earlyarrival(D); };
    bool latearrival(int i, const double D) const { return path[i].latearrival(D); };
    bool ontime(int i, const double D) const { return not earlyarrival(i, D) and not latearrival(i, D); };
    bool isdump(int i) const { return path[i].isdump(); };
    bool ispickup(int i) const { return path[i].ispickup(); };
    bool isdepot(int i) const { return path[i].isdepot(); };
    bool getCargo(int i) const { return path[i].getcargo(); };


};


#endif
