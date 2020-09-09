#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-module.h>
#include <ns3/ofswitch13-module.h>
#include <ns3/internet-apps-module.h>
#include <ns3/udp-client-server-helper.h>
#include <ns3/applications-module.h>
#include <vector>
#include <map>
#include <sstream>

#include "basic.h"

using namespace ns3;
using namespace std;

class DumpHelper {
public:
	DumpHelper(uint16_t node_number); 
    void dumpDpidList(const NodeContainer switches);
    void dumpIndexInfo(const nodeInfo_t* indexInfo);
    void dumpDpidInfo(const nodeInfo_t* dpidInfo);
    void dumpIndexAdj(double** const indexAdj);
    void dumpDpidAdj(double** const dpidAdj);
    void dumpDevPortMap(int** const devPortMap);
    void dumpDpidPortMap(int** const dpidPortMap);
private:
    uint16_t _node_number;
 
};