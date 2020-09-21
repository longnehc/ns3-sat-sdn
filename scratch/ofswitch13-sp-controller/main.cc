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

#include "ns3/ipv4-global-routing-helper.h"


#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "sp-ofswitch13-internal-helper.h"

#include <vector>
#include <map>
#include <sstream>

#include "basic.h"
#include "topology.h"
#include "sp-controller.h"
#include "satpos.h"
#include "dumphelper.h"


using namespace ns3;
using namespace std;


uint16_t simTime = 20;
uint16_t simBegin = 1;
uint16_t simEnd = 20;
bool verbose = false;
const uint16_t _nPlane = 8;
const uint16_t _nIndex = 9;
const uint16_t _nSat = _nPlane * _nIndex;
uint16_t _altitude = 780;
double _incl = 86.4;
double _latborder = 80; 

nodeInfo_t* indexInfo;
nodeInfo_t* dpidInfo;
NetDeviceContainer* switchPorts;
           
int** dpidPortMap;  
int** devPortMap;
double** indexAdj;
double** dpidAdj;  //dpid = index + 1

NodeContainer switches, hosts;
NetDeviceContainer serverPorts;
//ApplicationContainer apps;

Ipv4InterfaceContainer serverIpIfaces;

int NOW = 0;
 

void alloc(){ 
    indexInfo = new nodeInfo_t[_nSat];
    dpidInfo = new nodeInfo_t[_nSat + 1];
    switchPorts = new NetDeviceContainer[_nSat];

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
    delete[] switchPorts;
    
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
  
  TopoHelper topohelper(_nSat, _latborder, indexInfo, dpidInfo,  switchPorts, indexAdj, dpidAdj, dpidPortMap, devPortMap);
  
  vector<PolarSatPosition> satPositions = topohelper.SatNodeInit(_nPlane, _nIndex, _altitude, _incl);
  
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

      LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);
  // Enable checksum computations (required by OFSwitch13 module)
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Step3: Each satellite is bound with a host
  hosts.Create (_nSat);
  
  // Step4: Create switch nodes
  switches.Create (_nSat);
  //for(int i=0; i<_nSat; i++)
  //    switchPorts[i] = NetDeviceContainer();

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
  internet.Install (switches);

 
  //Step7: create datapath links  
  
  topohelper.SatLinkInit(satPositions, switches, _nPlane, _nIndex);

  //Step8: Create the controller node
  Ptr<Node> controllerNode = CreateObject<Node> ();

  //Step9: Configure SPcontroller for all switches
  Ptr<SPController> ctrl = CreateObject<SPController>();


  //Step8: Create the controller node
  Ptr<Node> superControllerNode = CreateObject<Node> ();

  //Step9: Configure controller and SPcontroller for all switches
  Ptr<SPController> spctrl = CreateObject<SPController>();


   // Set IPv4 server addresses
  Ipv4AddressHelper ipv4helpr;
  ipv4helpr.SetBase ("10.1.1.0", "255.255.255.0");
  serverIpIfaces = ipv4helpr.Assign (serverPorts);
 
 
 

/*
 


  Ptr<SPOFSwitch13InternalHelper> spof13Helper = CreateObject<SPOFSwitch13InternalHelper> ();
  //spof13Helper->InstallController (controllerNode, ctrl);
  //spof13Helper->InstallController (superControllerNode, spctrl);
  spof13Helper->InstallController (switches.Get(1), ctrl);
  //spof13Helper->InstallController (switches.Get(2), spctrl);
  for(int i=0; i<_nSat; i++){
      spof13Helper->InstallSwitch (switches.Get (i), switchPorts [i]);
  }
  //spof13Helper->SetChannelType(SPOFSwitch13InternalHelper::DEDICATEDCSMA);
  //spof13Helper->CreateOpenFlowChannels ();
  vector<int> ctIdv = {1};
  spof13Helper->CreateOpenFlowChannelsV2 (ctIdv); 
  

  ctrl->IsSC(false);   spctrl->IsSC(true); 

  topohelper.dpidInfoConstruct(switches);
  topohelper.dpidAdjConstruct(switches);
  topohelper.dpidPortMapConstruct(switches); 

//Dump information
    DumpHelper dumphelper(_nSat);
//  dumphelper.dumpDevPortMap(devPortMap);
//  dumphelper.dumpDpidList(switches);
//  dumphelper.dumpIndexInfo(indexInfo);
//  dumphelper.dumpDpidInfo(dpidInfo);
//  dumphelper.dumpIndexAdj(indexAdj);
//  dumphelper.dumpDpidAdj(dpidAdj);
//  dumphelper.dumpDevPortMap(devPortMap);
//  dumphelper.dumpDpidPortMap(dpidPortMap);

//-----

 
 
  
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
*/
  // Install UDP server on all nodes
  //trafficgen(0, 11, 4010, simBegin, simEnd, serverIpIfaces, hosts);
  //trafficgen(0, 11, 2000, simBegin, simEnd);
  //trafficgen(0, 11, 3000, simBegin, simEnd);

  FlowMonitorHelper flowmon;
  //Ptr<FlowMonitor> monitor = flowmon.Install (hosts); 

  Ptr<FlowMonitor> monitor = flowmon.Install(switches); 

  //Simulator::Schedule (MilliSeconds (1000), &TopoHelper::updatetopo, &topohelper, ctrl, spctrl, satPositions, switches, _nPlane, _nIndex);
  // Run the simulation 
  Simulator::Stop (Seconds (simTime)); 
  Simulator::Run ();

   monitor->SerializeToXmlFile("Log.xml", true, true);
   // 10. Print per flow statistics
   monitor->CheckForLostPackets ();
   Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
   FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
   for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
     {
      /*
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
      std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / simTime / 1000 / 1000  << " Mbps\n";
      std::cout << "  Delay: " << i->second.delaySum / 1000000 / i->second.rxPackets << "\n";
      std::cout << "  PLR: "<<(1.0*i->second.txPackets-i->second.rxPackets)/i->second.txPackets <<"\n"; 
      */
     }
  Simulator::Destroy ();
  dealloc();
}
