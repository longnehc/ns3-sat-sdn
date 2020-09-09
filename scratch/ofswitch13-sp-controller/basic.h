#ifndef _BASIC_H_  
#define _BASIC_H_ 
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-module.h> 
#include <ns3/internet-apps-module.h>
#include <ns3/udp-client-server-helper.h>
#include <ns3/applications-module.h>

#include "ns3/netanim-module.h"

#include <stdint.h>
#include <vector> 
#include <map>
#include <sstream> 
using namespace std;
using namespace ns3;

typedef uint64_t dpid_t;

typedef struct NodeInfo
{
    int plane;
    int index;
} nodeInfo_t;

string double2string(double a);
void alloc(nodeInfo_t* indexInfo, nodeInfo_t* dpidInfo, NetDeviceContainer* switchPorts, int** dpidPortMap,
  int** devPortMap, double** indexAdj, double** dpidAdj, int _nSat);
//void CreateUdpApp(int src, int dst, float datarate);
void trafficgen(int src, int dst, uint16_t port, double stime, double etime, Ipv4InterfaceContainer serverIpIfaces, NodeContainer hosts);


// Various constants
#define PI 3.1415926535897
#define MU 398601.2 // Greek Mu (km^3/s^2)
#define LIGHT 299793 // km/s
#define EARTH_PERIOD 86164 // seconds
#define EARTH_RADIUS 6378  // km
#define GEO_ALTITUDE 35786 // km
#define ATMOS_MARGIN 150 // km

#define DEG_TO_RAD(x) ((x) * PI/180)
#define RAD_TO_DEG(x) ((x) * 180/PI)
#define DISTANCE(s_x, s_y, s_z, e_x, e_y, e_z) (sqrt((s_x - e_x) * (s_x - e_x) \
                + (s_y - e_y) * (s_y - e_y) + (s_z - e_z) * (s_z - e_z)))

struct coordinate {
        double r;        // km
        double theta;    // radians
        double phi;      // radians
};

// Position types
#define POSITION_SAT 1
#define POSITION_SAT_POLAR 2
#define POSITION_SAT_GEO 3
#define POSITION_SAT_TERM 4

#endif
