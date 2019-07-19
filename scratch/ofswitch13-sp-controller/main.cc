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
uint16_t simTime = 10;
bool verbose = false;
uint16_t _nPlane = 8;
uint16_t _nIndex = 9;
uint16_t _nSat = _nPlane * _nIndex;
uint16_t _altitude = 780;
double _incl = 86.4;
double _latborder = 75;
vector<NetDeviceContainer> switchPorts (_nSat);

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
void updatePortMap(map<int, map<int, uint32_t>>& intPortMap, int dev1, int dev2, vector<NetDeviceContainer> switchPorts)
{
    
    uint32_t portNum = switchPorts[dev1].GetN();
    intPortMap[dev1][dev2] = portNum;
    std::cout<<"updatePortMap: dev_id= "<< dev1<<" to dev_id "<<dev2<<"'s port is "<<portNum<<std::endl;

    uint32_t portNum2 = switchPorts[dev2].GetN();
    intPortMap[dev2][dev1] = portNum2;
    std::cout<<"updatePortMap: dev_id= "<< dev2<<" to dev_id "<<dev1<<"'s port is "<<portNum2<<std::endl;
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

vector<PolarSatPosition> SatNodeInit(){
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

void buildLink(int src, int dst, double delay, map<int, map<int, uint32_t>>& devPortMap)
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

map<int, map<int, uint32_t>> SatLinkInit(vector<PolarSatPosition> satPositions){
  map<int, map<int, uint32_t>> devPortMap;
  SatGeometry sg;
 
  for(int i = 0; i < _nPlane; i++){
    for(int j = 0; j < _nIndex; j++){
      int node_index = i * _nIndex + j;
       //inter-plane isl
      int up_j = (j != _nIndex - 1) ? j + 1 : 0;
      int up_node_index = i * _nIndex + up_j;
 
      //up link construction
      double delay1 = sg.propdelay(satPositions[node_index].coord(0), satPositions[up_node_index].coord(0));
      buildLink(node_index, up_node_index, delay1, devPortMap);
      
      cout<<"Delays between ("<<i<<","<<j<<") and ("<<i<<","<<up_j<<" ) is "<<delay1<<endl;
      
      //intra-plane isl
      int right_i = (i != _nPlane - 1) ? i + 1 : 0;
      int right_node_index = right_i * _nIndex + j;
      double delay4 = sg.propdelay(satPositions[node_index].coord(0), satPositions[right_node_index].coord(0));
      buildLink(node_index, right_node_index, delay4, devPortMap);
      cout<<"Delays between ("<<i<<","<<j<<") and ("<<right_i<<","<<j<<" ) is "<<delay4<<endl;
    }
  }
  return devPortMap;
};

int
main (int argc, char *argv[])
{

  //LogComponentEnable ("OFSwitch13Port", LOG_LEVEL_INFO);
  //LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
  //LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);

  //create iridium topology
  //Step1: init sat positions
  vector<PolarSatPosition> satPositions = SatNodeInit();
  
 //Step3：

  // Configure command line parameters
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

  // Each satellite is bound with a host
  hosts.Create (_nSat);
  
  // Create switch nodes
  switches.Create (_nSat);
  for(int i=0; i<_nSat; i++)
      switchPorts[i] = NetDeviceContainer();

  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
  csmaHelper.SetChannelAttribute ("Delay", StringValue ("2ms"));
  
  NetDeviceContainer p2pDevices;
  NodeContainer pair;
 
  // connect host and switches
  for (int i=0; i<_nSat; i++){
    pair = NodeContainer (switches.Get (i), hosts.Get (i));
    p2pDevices = csmaHelper.Install (pair);
    switchPorts[i].Add(p2pDevices.Get(0));
    serverPorts.Add(p2pDevices.Get(1));
  }
  InternetStackHelper internet;
  internet.Install (hosts);

//
//csmaHelper.EnablePcap ("switch", switchPorts [0], true);
//csmaHelper.EnablePcap ("switch", switchPorts [2], true);
//

  std::cout<<"port number1: "<<switchPorts[0].GetN()<<std::endl;
/*
  CsmaHelper csmaH;
  csmaH.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
  csmaH.SetChannelAttribute ("Delay", StringValue ("2ms"));

  //add link n0---n1
  NodeContainer n0n1 = NodeContainer(switches.Get(0),switches.Get(1));  
  NetDeviceContainer p2pD1 = csmaH.Install(n0n1); 
  switchPorts[0].Add(p2pD1.Get(0));
  switchPorts[1].Add(p2pD1.Get(1));
  
  map<int, map<int, uint32_t>> intPortMap;
  updatePortMap(intPortMap, 0, 1, switchPorts);
  updatePortMap(intPortMap, 1, 0, switchPorts);

 //add link n1---n2
  NodeContainer n1n2 = NodeContainer(switches.Get(1),switches.Get(2));  
  NetDeviceContainer p2pD2 = csmaH.Install(n1n2); 
  switchPorts[1].Add(p2pD2.Get(0));
  switchPorts[2].Add(p2pD2.Get(1));

  updatePortMap(intPortMap, 1, 2, switchPorts);
  updatePortMap(intPortMap, 2, 1, switchPorts);
*/  

  //Step2: create datapath links 
  map<int, map<int, uint32_t>> devPortMap = SatLinkInit(satPositions); 
 
  // Create the controller node
  Ptr<Node> controllerNode = CreateObject<Node> ();

  // Configure SPcontroller for all switches
  Ptr<SPController> spCtrl = CreateObject<SPController>();

  // Configure the OpenFlow network domain
  Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper> ();
  of13Helper->InstallController (controllerNode, spCtrl);
  for(int i=0; i<_nSat; i++){
      of13Helper->InstallSwitch (switches.Get (i), switchPorts [i]);
  }
  of13Helper->CreateOpenFlowChannels ();

//TODO: 
  //std::cout<<"port number2: "<<switchPorts[0].GetN()<<std::endl;
  Ptr<OFSwitch13Device> ofdev0 = switches.Get(0)->GetObject<OFSwitch13Device>();
  dpid_t dpid0 = ofdev0->GetDpId();
 
  Ptr<OFSwitch13Device> ofdev1 = switches.Get(1)->GetObject<OFSwitch13Device>();
  dpid_t dpid1 = ofdev1->GetDpId();

  Ptr<OFSwitch13Device> ofdev2 = switches.Get(2)->GetObject<OFSwitch13Device>();
  dpid_t dpid2 = ofdev2->GetDpId();
  
  map<dpid_t, map<dpid_t, uint32_t>> dpidPortMap;
  
  //dpidPortMapConstruct(intPortMap, dpidPortMap, switches); 
  
  map<dpid_t, vector<dpid_t>> dpidAdj;

  vector<dpid_t> vec1;
  vec1.push_back(dpid1);
  dpidAdj[dpid0] = vec1;

  vector<dpid_t> vec2;
  vec2.push_back(dpid0);
  vec2.push_back(dpid2);
  dpidAdj[dpid1] = vec2;

  vector<dpid_t> vec3;
  vec3.push_back(dpid1);
  dpidAdj[dpid2] = vec3;


//-----



  // Set IPv4 server addresses
  Ipv4AddressHelper ipv4helpr;
  ipv4helpr.SetBase ("10.1.1.0", "255.255.255.0");
  serverIpIfaces = ipv4helpr.Assign (serverPorts);
 
  spCtrl->ImportNodes(switches);
  spCtrl->ImportServers(hosts);
  spCtrl->ImportDpidPortMap(dpidPortMap);
  spCtrl->ImportDpidAdj(dpidAdj);
  
  // Install UDP server on all nodes (port 11399)
  UdpServerHelper udpServerHelper (11399);
  ApplicationContainer serverApps = udpServerHelper.Install (hosts);
  
  //serverApps.Start (Seconds (1.0));
  //serverApps.Stop (Seconds (10.0));


  // Configure udp application between two hosts
  CreateUdpApp(0,2,1);
  
  //apps.Start (Seconds (2.0));
  //apps.Stop (Seconds (10.0));

  // Run the simulation
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
}
