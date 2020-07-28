/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 University of Campinas (Unicamp)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Luciano Chaves <luciano@lrc.ic.unicamp.br>
 *         Vitor M. Eichemberger <vitor.marge@gmail.com>
 *
 * Two hosts connected to different OpenFlow switches.
 * Both switches are managed by the default learning controller application.
 *
 *                       Learning Controller
 *                                |
 *                         +-------------+
 *                         |             |
 *                  +----------+     +----------+
 *       Host 0 === | Switch 0 | === | Switch 1 | === Host 1
 *                  +----------+     +----------+
 */

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

#include "sp-controller.h"
#include "satpos.h"

using namespace ns3;
using namespace std;

NodeContainer switches, hosts;
NetDeviceContainer serverPorts;
ApplicationContainer apps;

Ipv4InterfaceContainer serverIpIfaces;

int NOW = 0;
uint16_t simTime = 100;
uint16_t simBegin = 1;
uint16_t simEnd = 100;
bool verbose = false;
const uint16_t _nPlane = 8;
const uint16_t _nIndex = 9;
const uint16_t _nSat = _nPlane * _nIndex;
uint16_t _altitude = 780;
double _incl = 86.4;
double _latborder = 80;
vector<NetDeviceContainer> switchPorts (_nSat);



nodeInfo_t* indexInfo;
nodeInfo_t* dpidInfo;
 
//double indexAdj[_nSat][_nSat] = {-1};
//double dpidAdj[_nSat+1][_nSat+1] = {-1};            //dpid = index + 1

int** devPortMap;
int** dpidPortMap;

double** indexAdj;
double** dpidAdj;
 

void test(int* a){
  //cout<<"1111"<<endl;
  cout<<Simulator::Now ().GetMilliSeconds ()<<","<<*a<<endl;
  Simulator::Schedule (MilliSeconds (1), &test, a);
} 


void alloc(){
    indexInfo = new nodeInfo_t[_nSat];
    dpidInfo = new nodeInfo_t[_nSat + 1];

    devPortMap = new int* [_nSat];
    for(int i = 0; i < _nSat; i++)
        devPortMap[i] = new int[_nSat];

    dpidPortMap = new int* [_nSat + 1];
    for(int i = 0; i < _nSat + 1; i++)
        dpidPortMap[i] = new int[_nSat + 1];

    indexAdj = new double* [_nSat];
    for(int i = 0; i < _nSat; i++)
        indexAdj[i] = new double[_nSat];

    dpidAdj = new double* [_nSat + 1];
    for(int i = 0; i < _nSat+1; i++)
        dpidAdj[i] = new double[_nSat+1];


    for(int i = 0; i < _nSat; i++)
        for(int j = 0; j < _nSat; j++)
            devPortMap[i][j] = -1;
    for(int i = 0; i < _nSat + 1; i++)
        for(int j = 0; j < _nSat + 1; j++)
            dpidPortMap[i][j] = -1;

    for(int i = 0; i < _nSat; i++)
        for(int j = 0; j < _nSat; j++)
            indexAdj[i][j] = -1;
    for(int i = 0; i < _nSat + 1; i++)
        for(int j = 0; j < _nSat + 1; j++)
            dpidAdj[i][j] = -1;


}

void dealloc(){
    delete[] indexInfo;
    delete[] dpidInfo;
    
    for(int i = 0; i < _nSat; i++)
        delete[] devPortMap[i];
    delete[] devPortMap;

    for(int i = 0; i < _nSat+1; i++)
        delete[] dpidPortMap[i];
    delete[] dpidPortMap;
    
    for(int i = 0; i < _nSat; i++)
        delete[] indexAdj[i];
    delete[] indexAdj;

    for(int i = 0; i < _nSat+1; i++)
        delete[] dpidAdj[i];
    delete[] dpidAdj;
}

dpid_t getDpidByIndex(int i)
{
    Ptr<OFSwitch13Device> ofdev = switches.Get(i)->GetObject<OFSwitch13Device>();
    return ofdev->GetDpId();
}

void dpidInfoConstruct(nodeInfo_t* indexInfo, nodeInfo_t* dpidInfo, uint16_t nSat)
{
    for(int i = 0; i < nSat; i++){
        dpid_t _dpid = getDpidByIndex(i);
        dpidInfo[_dpid] = indexInfo[i];
    }
}

void dpidAdjConstruct(double** indexAdj, double** dpidAdj, uint16_t nSat)
{  
    for(int i = 0; i < nSat; i++){
        for(int j = 0; j < nSat; j++){
            dpid_t dpid_i = getDpidByIndex(i);
            dpid_t dpid_j = getDpidByIndex(j);
            dpidAdj[dpid_i][dpid_j] = indexAdj[i][j];
        }
    }
}

void dpidPortMapConstruct(int** devPortMap,int** dpidPortMap, uint16_t nSat)
{  
    for(int i = 0; i < nSat; i++){
        for(int j = 0; j < nSat; j++){
            dpid_t dpid_i = getDpidByIndex(i);
            dpid_t dpid_j = getDpidByIndex(j);
            dpidPortMap[dpid_i][dpid_j] = devPortMap[i][j];
        }
    }
}

void dpidPortMapConstruct(map<int, map<int, uint32_t>> intPortMap, map<dpid_t, map<dpid_t, uint32_t>>& dpidPortMap, NodeContainer switches)
{
    
    auto it = intPortMap.begin();
    while(it != intPortMap.end()){
        int switch0 = it->first;
        Ptr<OFSwitch13Device> ofdev0 = switches.Get(switch0)->GetObject<OFSwitch13Device>();
        dpid_t dpid0 = ofdev0->GetDpId(); 
        map<dpid_t, uint32_t> innerMap;
        map<int, uint32_t> im = it->second;
        auto inner_it = im.begin();
        while(inner_it != im.end()){
            int switchi = inner_it -> first;
            uint32_t portNum = inner_it -> second;
            Ptr<OFSwitch13Device> ofdevi = switches.Get(switchi)->GetObject<OFSwitch13Device>();
            dpid_t dpidi = ofdevi->GetDpId(); 
            dpidPortMap[dpid0][dpidi] = portNum;
            std::cout<<"dpidPortMapConstruct: Dpid= "<< dpid0<<" to dpid "<<dpidi<<"'s port is "<<portNum<<std::endl;
            inner_it++;
        }
        it++;
    }
}
void updatePortMap(int** devPortMap, int dev1, int dev2, vector<NetDeviceContainer> switchPorts)
{
    
    uint32_t portNum = switchPorts[dev1].GetN();
    devPortMap[dev1][dev2] = portNum;
//    std::cout<<"updatePortMap: dev_id= "<< dev1<<" to dev_id "<<dev2<<"'s port is "<<portNum<<std::endl;

    uint32_t portNum2 = switchPorts[dev2].GetN();
    devPortMap[dev2][dev1] = portNum2;
//    std::cout<<"updatePortMap: dev_id= "<< dev2<<" to dev_id "<<dev1<<"'s port is "<<portNum2<<std::endl;
}

//datarate is Mbps
void
CreateUdpApp(int src, int dst, float datarate)
{
    Address remoteServerAddr = Address(serverIpIfaces.GetAddress(dst));
    std::cout<<"remote addr:" << remoteServerAddr<<std::endl;
    UdpClientHelper helper(remoteServerAddr, 11399);
    helper.SetAttribute("Interval", TimeValue(MilliSeconds(1/datarate)));
    NodeContainer clients (hosts.Get(src));
    ApplicationContainer curapp = helper.Install (clients);
    apps.Add(curapp.Get(0));
}

vector<PolarSatPosition> 
SatNodeInit(){
  vector<PolarSatPosition> satPositions;
  SatGeometry sg;
  for (int i=0; i<_nPlane; i++){
    for(int j=0; j<_nIndex; j++){
      double lon = 22.5 * i;    // double lon = 31.6*i; 31.6 for iridium, 6 planes - 2pi, so 180 / 8 = 22.5
      double alpha =  fmod(5 * i + 40 * j, 360); //360/(8*9)=5 360 / 9 = 40
      PolarSatPosition psp = PolarSatPosition(_altitude, _incl, lon, alpha, i, j);
      //cout<<"Inserting node "<<i<<","<<j<<" lon: "<<lon<<" alpha: "<< alpha<<endl;
      satPositions.push_back(psp);
      //cout<<"The lat is: "<<RAD_TO_DEG(sg.get_latitude(psp.coord(0)))<<", the long is "<<RAD_TO_DEG(sg.get_longitude(psp.coord(0),0))<<endl;
      nodeInfo_t nit;
      nit.plane = i;
      nit.index = j;
      indexInfo[i * _nIndex + j] = nit;
    }
  }
  return satPositions;
}

string double2string(double a){
  stringstream strStream;  
  strStream << a; 
  string s = strStream.str();  
  return s;
}

bool inPolar(coordinate a)
{
    SatGeometry sg;
    bool ret = false;
    if (RAD_TO_DEG(sg.get_latitude(a)) > _latborder || RAD_TO_DEG(sg.get_latitude(a)) < -_latborder)
        ret = true;
    //if(ret)
    //    cout<<"The invalid latitude is: "<<RAD_TO_DEG(sg.get_latitude(a))<<endl;
    return ret;
}

void buildLink(int src, int dst, double delay, int** devPortMap)
{
      CsmaHelper csmaH;
      csmaH.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
      csmaH.SetChannelAttribute ("Delay", StringValue (double2string(delay)+"ms"));
      //add link n0---n1
      NodeContainer nc = NodeContainer(switches.Get(src),switches.Get(dst));  
      NetDeviceContainer p2pD = csmaH.Install(nc); 
      switchPorts[src].Add(p2pD.Get(0));
      switchPorts[dst].Add(p2pD.Get(1));
      updatePortMap(devPortMap, src, dst, switchPorts);
}

void SatLinkInit(vector<PolarSatPosition> satPositions, int** devPortMap){
  SatGeometry sg;
 
  for(int i = 0; i < _nPlane; i++){
    for(int j = 0; j < _nIndex; j++){
      int node_index = i * _nIndex + j;
       //inter-plane isl
      int up_j = (j != _nIndex - 1) ? j + 1 : 0;
      int up_node_index = i * _nIndex + up_j;
 
      //if(!inPolar(satPositions[node_index].coord(0)) && !inPolar(satPositions[up_node_index].coord(0))) {
        double delay1 = sg.propdelay(satPositions[node_index].coord(0), satPositions[up_node_index].coord(0));
        buildLink(node_index, up_node_index, delay1 * 1000, devPortMap);
        indexAdj[node_index][up_node_index] = delay1 * 1000;
        indexAdj[up_node_index][node_index] = delay1 * 1000;
       // cout<<"Delays between" <<node_index<<" "<<up_node_index<<"("<<i<<","<<j<<") -> ("<<i<<","<<up_j<<" ) is "<<delay1<<endl;
      //} 
      
      
      //intra-plane isl
      int right_i = (i != _nPlane - 1) ? i + 1 : 0;
      int right_node_index = right_i * _nIndex + j;
      if(!inPolar(satPositions[node_index].coord(0)) && !inPolar(satPositions[right_node_index].coord(0))) {
          double delay4 = sg.propdelay(satPositions[node_index].coord(0), satPositions[right_node_index].coord(0));
          buildLink(node_index, right_node_index, delay4 * 1000, devPortMap);
          indexAdj[node_index][right_node_index] = delay4 * 1000;
          indexAdj[right_node_index][node_index] = delay4 * 1000;
       //   cout<<"Delays between" <<node_index<<" "<<right_node_index<<"("<<i<<","<<j<<") -> ("<<right_i<<","<<j<<" ) is "<<delay4<<endl;
      }
     
    }
  }
}

void dumpDpidlist()
{
    for(uint32_t i = 0; i < switches.GetN(); i++) {
        Ptr<OFSwitch13Device> ofdev = switches.Get(i)->GetObject<OFSwitch13Device>();
        cout<<"dumpDpidlist: dpid of dev "<<i <<" is "<<ofdev->GetDpId()<<endl;
    }
}
 
void dumpIndexinfo()
{
    for(uint32_t i = 0; i < _nSat; i++) {
        nodeInfo_t a = indexInfo[i];
        cout<<"dumpIndexinfo: dev: "<<i <<", plane = "<<a.plane<<", index= "<<a.index<<endl;
    }
} 

void dumpDpidinfo()
{
    for(uint32_t i = 1; i < _nSat + 1; i++) {
        nodeInfo_t a = dpidInfo[i];
        cout<<"dumpDpidinfo: dpid: "<<i <<", plane = "<<a.plane<<", index= "<<a.index<<endl;
    }

}

void dumpindexAdj()
{
    for(uint32_t i = 0; i < _nSat; i++) {
        for(uint32_t j = 0; j < _nSat; j++){
            if(indexAdj[i][j] != -1){
                cout<<"dumpindexAdj: dev: "<<i <<" to dev: "<<j<<" cost= "<<indexAdj[i][j]<<endl;
            }
        }
    }
} 
 
void dumpdpidAdj()
{
   
    for(uint32_t i = 1; i < _nSat+1; i++) {
        bool find = false;
        for(uint32_t j = 1; j < _nSat+1; j++){
            if(dpidAdj[i][j] != -1){
                find =true;
                cout<<"dumpindexAdj: dpid: "<<i <<" to dpid: "<<j<<" cost= "<<dpidAdj[i][j]<<endl;
            }
        }
        if(!find)  cout<<"dumpindexAdj: dpid: "<<i <<" has no neighbor!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    }
  

} 

void dumpDevPortMap()
{
    for(uint32_t i = 0; i < _nSat; i++) {
        for(uint32_t j = 0; j < _nSat; j++){
            if(devPortMap[i][j] != -1){
                cout<<"dumpDevPortMap: dev: "<<i <<" to dev: "<<j<<" port= "<<devPortMap[i][j]<<endl;
            }
        }
    }
}

void dumpDpidPortMap()
{
    for(uint32_t i = 1; i < _nSat+1; i++) {
        for(uint32_t j = 1; j < _nSat+1; j++){
            if(dpidPortMap[i][j] != -1){
                cout<<"dumpDpidPortMap: dpid: "<<i <<" to dpid: "<<j<<" port= "<<dpidPortMap[i][j]<<endl;
            }
        }
    }
}

void trafficgen(int src, int dst, uint16_t port, double stime, double etime){

  cout<<"etime="<<etime<<endl;
   UdpServerHelper server (port);
   ApplicationContainer apps = server.Install (hosts.Get (dst));
   apps.Start (Seconds (stime));
   apps.Stop (Seconds (etime));
 
 //
 // Create one UdpClient application to send UDP datagrams from node zero to
 // node one.
 //
   uint32_t MaxPacketSize = 1024;
   Time interPacketInterval = Seconds (0.05);
   uint32_t maxPacketCount = 10000;
   Address remoteServerAddr = Address(serverIpIfaces.GetAddress(dst));
   UdpClientHelper client (remoteServerAddr, port);
   client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
   client.SetAttribute ("Interval", TimeValue (interPacketInterval));
   client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
   apps = client.Install (hosts.Get (src));
   apps.Start (Seconds (stime));
   apps.Stop (Seconds (etime));
}

void updatetopo(Ptr<SPController> ctrl, Ptr<SPController> spctrl, vector<PolarSatPosition> satPositions){
  int now = Simulator::Now().GetSeconds();
  cout<<"Update topology invoked at "<<now<<endl;
  SatGeometry sg;

  for(int i = 0; i < _nSat; i++)
        for(int j = 0; j < _nSat; j++)
            indexAdj[i][j] = -1;
  for(int i = 0; i < _nSat + 1; i++)
        for(int j = 0; j < _nSat + 1; j++)
            dpidAdj[i][j] = -1;

  for(int i = 0; i < _nPlane; i++){
    for(int j = 0; j < _nIndex; j++){
      int node_index = i * _nIndex + j;
       //inter-plane isl
      int up_j = (j != _nIndex - 1) ? j + 1 : 0;
      int up_node_index = i * _nIndex + up_j;
      //if(!inPolar(satPositions[node_index].coord(0)) && !inPolar(satPositions[up_node_index].coord(0))) {
      double delay1 = sg.propdelay(satPositions[node_index].coord(now), satPositions[up_node_index].coord(now));
      indexAdj[node_index][up_node_index] = delay1 * 1000;
      indexAdj[up_node_index][node_index] = delay1 * 1000;
       // cout<<"Delays between" <<node_index<<" "<<up_node_index<<"("<<i<<","<<j<<") -> ("<<i<<","<<up_j<<" ) is "<<delay1<<endl;
      //} 
      //intra-plane isl
      int right_i = (i != _nPlane - 1) ? i + 1 : 0;
      int right_node_index = right_i * _nIndex + j;
      if(!inPolar(satPositions[node_index].coord(now)) && !inPolar(satPositions[right_node_index].coord(now))) {
          double delay4 = sg.propdelay(satPositions[node_index].coord(now), satPositions[right_node_index].coord(now));
          indexAdj[node_index][right_node_index] = delay4 * 1000;
          indexAdj[right_node_index][node_index] = delay4 * 1000;
       //   cout<<"Delays between" <<node_index<<" "<<right_node_index<<"("<<i<<","<<j<<") -> ("<<right_i<<","<<j<<" ) is "<<delay4<<endl;
      }
    }
  }
  dpidAdjConstruct(indexAdj, dpidAdj, _nSat);

  ctrl->ImportDpidAdj(dpidAdj, _nSat + 1);
  spctrl->ImportDpidAdj(dpidAdj, _nSat + 1);

  Simulator::Schedule (MilliSeconds (1000), updatetopo, ctrl, spctrl, satPositions);
}

void ld(int scnum, int cnum, Ptr<SPController> ctrl, Ptr<SPController> spctrl, int simBegin, int simEnd){
  map<int,int> c2sc, s2c;
  //int scnum =  1;
  //int cnum = 2;
  c2sc[0] = 0;
  c2sc[1] = 0;
  for(int j = 0; j < _nSat; j++){
    if(j <= _nSat / 2){
      s2c[j+1] = 0;   //dpid = dev + 1
    }
    else{
      s2c[j+1] = 1;
    }
  }
  
  ctrl-> ImportDomainConfig(cnum,scnum, s2c, c2sc);
  spctrl-> ImportDomainConfig(cnum,scnum, s2c, c2sc);

  for(int i = simBegin; i < simEnd; i++){
     ctrl->ImportCLocation(i,0,5);
     ctrl->ImportCLocation(i,1,11);
     ctrl->ImportSCLocation(i,0,17);
     spctrl->ImportCLocation(i,0,5);
     spctrl->ImportCLocation(i,1,11);
     spctrl->ImportSCLocation(i,0,17);
  }
}

void dis(int scnum, int cnum, Ptr<SPController> ctrl, Ptr<SPController> spctrl, int simBegin, int simEnd){
  map<int,int> c2sc, s2c;
  //int scnum =  1;
  //int cnum = 2;
  c2sc[0] = 0;
  c2sc[1] = 0;
  for(int j = 0; j < _nSat; j++){
    double cost1 = 0, cost2 = 0;
    vector<dpid_t> path1;
    if(j+1 == 5) s2c[j+1] = 0;
    else if (j+1 == 11) s2c[j+1] = 1;
    else{
      ctrl->calPath(j+1, 5, path1);
      for(uint32_t i = 0; i < path1.size()-1; i++){
             if(dpidAdj[path1[i]][path1[i+1]] != -1) 
                 cost1 += dpidAdj[path1[i]][path1[i+1]];
             else
                NS_ABORT_MSG ("Path calulation error.");
      }

      vector<dpid_t> path2;
      ctrl->calPath(j+1, 11, path2);
      for(uint32_t i = 0; i < path2.size()-1; i++){
             if(dpidAdj[path2[i]][path2[i+1]] != -1) 
                 cost2 += dpidAdj[path2[i]][path2[i+1]];
             else
                NS_ABORT_MSG ("Path calulation error.");
      }

      if(cost1 < cost2)
        s2c[j+1] = 0;
      else 
        s2c[j+1] = 1;
    }
  }
  
  ctrl-> ImportDomainConfig(cnum,scnum, s2c, c2sc);
  spctrl-> ImportDomainConfig(cnum,scnum, s2c, c2sc);

  for(int i = simBegin; i < simEnd; i++){
     ctrl->ImportCLocation(i,0,5);
     ctrl->ImportCLocation(i,1,11);
     ctrl->ImportSCLocation(i,0,17);
     spctrl->ImportCLocation(i,0,5);
     spctrl->ImportCLocation(i,1,11);
     spctrl->ImportSCLocation(i,0,17);
  }

}

int
main (int argc, char *argv[])
{

  //LogComponentEnable ("OFSwitch13Port", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
  LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);

  //create iridium topology
  alloc();
  //Step1: init sat positions
  vector<PolarSatPosition> satPositions = SatNodeInit();
  
  // Step2: Configure command line parameters
  CommandLine cmd;
  cmd.AddValue ("simTime", "Simulation time (seconds)", simTime);
  cmd.AddValue ("verbose", "Enable verbose output", verbose);
  cmd.AddValue ("now", "Set time", NOW);
  cmd.Parse (argc, argv);

  if (verbose)
    {
      OFSwitch13Helper::EnableDatapathLogs ();
      LogComponentEnable ("OFSwitch13Interface", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13Device", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13Port", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13Queue", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13SocketHandler", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13Controller", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13LearningController", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13Helper", LOG_LEVEL_ALL);
      LogComponentEnable ("OFSwitch13InternalHelper", LOG_LEVEL_ALL);
    }

  // Enable checksum computations (required by OFSwitch13 module)
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Step3: Each satellite is bound with a host
  hosts.Create (_nSat);
  
  // Step4: Create switch nodes
  switches.Create (_nSat);
  for(int i=0; i<_nSat; i++)
      switchPorts[i] = NetDeviceContainer();

  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
  csmaHelper.SetChannelAttribute ("Delay", StringValue ("2ms"));
  
  NetDeviceContainer p2pDevices;
  NodeContainer pair;
 
  // Stepp6: connect host and switches
  for (int i=0; i<_nSat; i++){
    pair = NodeContainer (switches.Get (i), hosts.Get (i));
    p2pDevices = csmaHelper.Install (pair);
    switchPorts[i].Add(p2pDevices.Get(0));
    serverPorts.Add(p2pDevices.Get(1));
  }
  InternetStackHelper internet;
  internet.Install (hosts);

 
  //Step7: create datapath links 
  SatLinkInit(satPositions, devPortMap); 
 
  //Step8: Create the controller node
  Ptr<Node> controllerNode = CreateObject<Node> ();

  //Step9: Configure SPcontroller for all switches
  Ptr<SPController> ctrl = CreateObject<SPController>();


  //Step8: Create the controller node
  Ptr<Node> superControllerNode = CreateObject<Node> ();

  //Step9: Configure controller and SPcontroller for all switches
  Ptr<SPController> spctrl = CreateObject<SPController>();


  Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper> ();
  of13Helper->InstallController (controllerNode, ctrl);
  of13Helper->InstallController (superControllerNode, spctrl);
  ctrl->IsSC(false);   spctrl->IsSC(true); 
  for(int i=0; i<_nSat; i++){
      of13Helper->InstallSwitch (switches.Get (i), switchPorts [i]);
  }
  of13Helper->CreateOpenFlowChannels ();

  dpidInfoConstruct(indexInfo, dpidInfo, _nSat);
  dpidAdjConstruct(indexAdj, dpidAdj, _nSat);
  dpidPortMapConstruct(devPortMap, dpidPortMap, _nSat);

//  dumpDpidlist();
//  dumpIndexinfo();
//  dumpDpidinfo();
//  dumpindexAdj();
//  dumpdpidAdj();
//  dumpDevPortMap();
//  dumpDpidPortMap();

//-----

  // Set IPv4 server addresses
  Ipv4AddressHelper ipv4helpr;
  ipv4helpr.SetBase ("10.1.1.0", "255.255.255.0");
  serverIpIfaces = ipv4helpr.Assign (serverPorts);
 
  ctrl->ImportNodes(switches);
  ctrl->ImportServers(hosts);

  ctrl->ImportBasicInfo(_nPlane, _nIndex);
  ctrl->ImportDpidInfo(dpidInfo, _nSat + 1);
  ctrl->ImportDpidPortMap(dpidPortMap, _nSat + 1);
  ctrl->ImportDpidAdj(dpidAdj, _nSat + 1);
  ctrl->ImportFlag(0000);


  spctrl->ImportNodes(switches);
  spctrl->ImportServers(hosts);

  spctrl->ImportBasicInfo(_nPlane, _nIndex);
  spctrl->ImportDpidInfo(dpidInfo, _nSat + 1);
  spctrl->ImportDpidPortMap(dpidPortMap, _nSat + 1);
  spctrl->ImportDpidAdj(dpidAdj, _nSat + 1);
  spctrl->ImportFlag(1111);


    //Step10: Configure the OpenFlow network domain
   //dis(1, 2, ctrl, spctrl, simBegin, simEnd);
   ld(1,2,ctrl, spctrl, simBegin, simEnd);

  // Install UDP server on all nodes
  trafficgen(0, 11, 4010, simBegin, simEnd);

  Simulator::Schedule (MilliSeconds (1000), updatetopo, ctrl, spctrl, satPositions);
  // Run the simulation
  //int a = 1;
  Simulator::Stop (Seconds (simTime));
  //Simulator::Schedule (MilliSeconds (1), &test, &a);
  Simulator::Run ();
  Simulator::Destroy ();
  dealloc();
}
