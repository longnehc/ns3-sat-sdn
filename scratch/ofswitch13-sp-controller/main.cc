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

#include "sp-controller.h"
#include "satpos.h"

using namespace ns3;
using namespace std;

NodeContainer switches, servers;
NetDeviceContainer serverPorts;
ApplicationContainer apps;
Ipv4InterfaceContainer serverIpIfaces;

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

  //create iridium topology
  uint16_t nPlane = 6;
  uint16_t nIndex = 11;
  uint16_t nSat = nPlane * nIndex;
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

  //create intra-plane link
  for(int i=0; i<nPlane; i++){
  
  }

  //create inter-plane link
  for(int i=0; i<nPlane; i++){
  
  }

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
 
  // Set IPv4 server addresses
  Ipv4AddressHelper ipv4helpr;
  ipv4helpr.SetBase ("10.1.1.0", "255.255.255.0");
  serverIpIfaces = ipv4helpr.Assign (serverPorts);
  
  spCtrl->ImportNodes(switches);
  spCtrl->ImportServers(servers);
  
  // Install UDP server on all nodes (port 11399)
  UdpServerHelper udpServerHelper (11399);
  ApplicationContainer serverApps = udpServerHelper.Install (servers);
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));


  // Configure udp application between two hosts
  CreateUdpApp(0,22,1);
  apps.Start (Seconds (2.0));
  apps.Stop (Seconds (10.0));

  // Run the simulation
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
}
