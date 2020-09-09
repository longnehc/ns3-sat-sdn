#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-module.h>
#include <ns3/ofswitch13-module.h>
#include <ns3/internet-apps-module.h>
#include <ns3/udp-client-server-helper.h>
#include <ns3/applications-module.h>
#include <iostream>
#include "basic.h"
#include "satpos.h"

#include "sp-controller.h"

using namespace ns3;
using namespace std;

class TopoHelper {
public:
	//TopoHelper(double latborder);
	TopoHelper(uint16_t nSat, double latborder, nodeInfo_t* indexInfo, nodeInfo_t* dpidInfo,  NetDeviceContainer* switchPorts, 
        double** indexAdj, double** dpidAdj, int** dpidPortMap, int** devPortMap);
	vector<PolarSatPosition> SatNodeInit(int _nPlane, int _nIndex, uint16_t _altitude, double _incl);
    void SatLinkInit(vector<PolarSatPosition> satPositions, NodeContainer switches, int _nPlane, int _nIndex);
    void buildLink(int src, int dst, double delay, NodeContainer switches);
    void updatePortMap(int dev1, int dev2);
    dpid_t getDpidByIndex(int i, NodeContainer switches);
    void dpidInfoConstruct(NodeContainer switches);
    void dpidAdjConstruct(NodeContainer switches);
    void dpidPortMapConstruct(NodeContainer switches);
    void updatetopo(Ptr<SPController> ctrl, Ptr<SPController> spctrl, vector<PolarSatPosition> satPositions, NodeContainer switches, int _nPlane, int _nIndex);
private:
    uint16_t _nSat;
	double _latborder;
    nodeInfo_t* _indexInfo;
    nodeInfo_t* _dpidInfo;
    NetDeviceContainer* _switchPorts;
    int** _dpidPortMap;
    int** _devPortMap;
    double** _indexAdj;
    double** _dpidAdj;

};