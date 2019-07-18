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

#include "sp-controller.h"
#include "satpos.h"

using namespace ns3;
using namespace std;

NodeContainer switches, servers;
NetDeviceContainer serverPorts;
ApplicationContainer apps;
Ipv4InterfaceContainer serverIpIfaces;


void updatePortMap(map<dpid_t, map<dpid_t, uint32_t>>& dpidPortMap, dpid_t dpid1, dpid_t dpid2)
{
    auto it = dpidPortMap.find(dpid1);
    if(it != dpidPortMap.end()) {
        uint32_t portNum = (it->second).size() + 2;
        pair<dpid_t,uint32_t> value(dpid2, portNum);
        dpidPortMap[dpid1].insert(value);
        std::cout<<"updatePortMap: Dpid= "<< dpid1<<" to dpid "<<dpid2<<"'s port is "<<portNum<<std::endl;
    } else {
        map<dpid_t, uint32_t> initial_map;
        pair<dpid_t,uint32_t> value(dpid2,2);
        initial_map.insert(value);
        pair<dpid_t, map<dpid_t, uint32_t>> temp_value(dpid1, initial_map);
        dpidPortMap.insert(temp_value);
        std::cout<<"updatePortMap(0): Dpid= "<< dpid1<<" to dpid "<<dpid2<<"'s port is "<<2<<std::endl;
    }
}

//datarate is Mbps
void
CreateUdpApp(int src, int dst, float datarate)
{
    Address remoteServerAddr = Address(serverIpIfaces.GetAddress(dst));
   

    std::cout<<"remote addr:" << remoteServerAddr<<std::endl;
    
    UdpClientHelper helper(remoteServerAddr);
    helper.SetAttribute("Interval", TimeValue(MilliSeconds(1/datarate)));
    NodeContainer clients (servers.Get(src));
    ApplicationContainer curapp = helper.Install (clients);
    apps.Add(curapp.Get(0));
}

int
main (int argc, char *argv[])
{
  int NOW = 0;
  uint16_t simTime = 10;
  bool verbose = false;
  LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
  //create iridium topology
  uint16_t nPlane = 6;
  uint16_t nIndex = 11;
  uint16_t nSat = nPlane * nIndex;
  //uint16_t nSat = 3;
  uint16_t altitude = 780;
  double incl = 86.4;

  vector<PolarSatPosition> satPositions;
  for (int i=0; i<nPlane; i++){
      double lon = 31.6*i;
      double alpha0 = 0;
      if (i & 1)
          alpha0 = 180/11;
      cout<<altitude<<" "<<lon<<endl;
      for(int j=0; j<nIndex; j++){
          satPositions.push_back(PolarSatPosition(altitude, incl, lon, alpha0+j*360/11, i));
      }
  }

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

  // Each satellite is bound with a server node
  servers.Create (nSat);
  
  // Create switch nodes
  switches.Create (nSat);
  vector<NetDeviceContainer> switchPorts (nSat);
  for(int i=0; i<nSat; i++)
      switchPorts[i] = NetDeviceContainer();

  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
  csmaHelper.SetChannelAttribute ("Delay", StringValue ("2ms"));
  
  NetDeviceContainer p2pDevices;
  NodeContainer pair;
 
  // connect server and switches
  for (int i=0; i<nSat; i++){
    pair = NodeContainer (switches.Get (i), servers.Get (i));
    p2pDevices = csmaHelper.Install (pair);
    switchPorts[i].Add(p2pDevices.Get(0));
    serverPorts.Add(p2pDevices.Get(1));
  }
  InternetStackHelper internet;
  internet.Install (servers);

//
csmaHelper.EnablePcap ("switch", switchPorts [0], true);
csmaHelper.EnablePcap ("switch", switchPorts [1], true);
//

  //create intra-plane link
  for(int i=0; i<nPlane; i++){
  
  }

  //create inter-plane link
  for(int i=0; i<nPlane; i++){
  
  }


  CsmaHelper csmaH;
  csmaH.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
  csmaH.SetChannelAttribute ("Delay", StringValue ("2ms"));

  //add link n0---n1
  NodeContainer n0n1 = NodeContainer(switches.Get(0),switches.Get(1));  
  NetDeviceContainer p2pD1 = csmaH.Install(n0n1); 
  switchPorts[0].Add(p2pD1.Get(0));
  switchPorts[1].Add(p2pD1.Get(1));

 //add link n1---n2
  NodeContainer n1n2 = NodeContainer(switches.Get(1),switches.Get(2));  
  NetDeviceContainer p2pD2 = csmaH.Install(n1n2); 
  switchPorts[1].Add(p2pD2.Get(0));
  switchPorts[2].Add(p2pD2.Get(1));

  // Create the controller node
  Ptr<Node> controllerNode = CreateObject<Node> ();

  // Configure SPcontroller for all switches
  Ptr<SPController> spCtrl = CreateObject<SPController>();

  // Configure the OpenFlow network domain
  Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper> ();
  of13Helper->InstallController (controllerNode, spCtrl);
  for(int i=0; i<nSat; i++){
      of13Helper->InstallSwitch (switches.Get (i), switchPorts [i]);
  }
  of13Helper->CreateOpenFlowChannels ();


  Ptr<OFSwitch13Device> ofdev0 = switches.Get(0)->GetObject<OFSwitch13Device>();
  dpid_t dpid0 = ofdev0->GetDpId();
 
  Ptr<OFSwitch13Device> ofdev1 = switches.Get(1)->GetObject<OFSwitch13Device>();
  dpid_t dpid1 = ofdev1->GetDpId();

  Ptr<OFSwitch13Device> ofdev2 = switches.Get(2)->GetObject<OFSwitch13Device>();
  dpid_t dpid2 = ofdev2->GetDpId();
  
  map<dpid_t, map<dpid_t, uint32_t>> dpidPortMap;
  map<dpid_t, vector<dpid_t>> dpidAdj;

  updatePortMap(dpidPortMap, dpid0, dpid1);
  updatePortMap(dpidPortMap, dpid1, dpid0);
  updatePortMap(dpidPortMap, dpid1, dpid2);
  updatePortMap(dpidPortMap, dpid2, dpid1);
  /*
  dpidPortMap[dpid2] = tmp4;
*/
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


  // Set IPv4 server addresses
  Ipv4AddressHelper ipv4helpr;
  ipv4helpr.SetBase ("10.1.1.0", "255.255.255.0");
  serverIpIfaces = ipv4helpr.Assign (serverPorts);
 
  spCtrl->ImportNodes(switches);
  spCtrl->ImportServers(servers);
  spCtrl->ImportDpidPortMap(dpidPortMap);
  spCtrl->ImportDpidAdj(dpidAdj);
  
  // Install UDP server on all nodes (port 11399)
  UdpServerHelper udpServerHelper (0);
  ApplicationContainer serverApps = udpServerHelper.Install (servers);
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));


  // Configure udp application between two hosts
  CreateUdpApp(0,1,1);
  apps.Start (Seconds (2.0));
  apps.Stop (Seconds (10.0));

  // Run the simulation
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
}
