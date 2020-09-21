/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Piotr Gawlowicz
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
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 */

#include "ns3/core-module.h"
#include "ns3/opengym-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "mygym.h"
#include <fstream>
#include <time.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("OpenGym");

int
main (int argc, char *argv[])
{
  // Parameters of the scenario
  uint32_t simSeed = 1;
  double simulationTime = 20; //seconds
  double envStepTime = 0.1; //seconds, ns3gym env step time interval
  uint32_t openGymPort = 15555;
  uint32_t testArg = 0;
  uint32_t xSize = 3, ySize = 3;
  std::string link_bandwidth = "50Mbps";
  std::string link_delay = "10ms";

  CommandLine cmd;
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
  // optional parameters
  cmd.AddValue ("simTime", "Simulation time in seconds. Default: 100s", simulationTime);
  cmd.AddValue ("testArg", "Extra simulation argument. Default: 0", testArg);
  cmd.AddValue ("xsize", "Number of nodes in a row grid", xSize);
  cmd.AddValue ("ysize", "Number of rows in a grid", ySize);
  cmd.AddValue ("link_bandwidth", "Link bandwidth", link_bandwidth);
  cmd.AddValue ("link_delay", "Link delay", link_delay);
  cmd.Parse (argc, argv);

  NS_LOG_UNCOND("Ns3Env parameters:");
  NS_LOG_UNCOND("--simulationTime: " << simulationTime);
  NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  NS_LOG_UNCOND("--envStepTime: " << envStepTime);
  NS_LOG_UNCOND("--seed: " << simSeed);
  NS_LOG_UNCOND("--testArg: " << testArg);
  NS_LOG_DEBUG ("Grid:" << xSize << "*" << ySize);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (simSeed);

  // OpenGym Env
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (openGymPort);
  Ptr<MyGymEnv> myGymEnv = CreateObject<MyGymEnv> (Seconds(envStepTime));
  myGymEnv->SetOpenGymInterface(openGym);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue(link_bandwidth));
  pointToPoint.SetChannelAttribute ("Delay", StringValue(link_delay));

  // Create Grid
  PointToPointGridHelper grid (xSize, ySize, pointToPoint);

  // Install stack on Grid
  InternetStackHelper stack;
  grid.InstallStack (stack);

  // Assign Addresses to Grid
  grid.AssignIpv4Addresses (Ipv4AddressHelper ("10.1.1.0", "255.255.255.0"),
                            Ipv4AddressHelper ("10.2.1.0", "255.255.255.0"));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // Flow monitor
  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> monitor = flowHelper.InstallAll ();
  
  // Init Gym
  myGymEnv->SetTopo(&grid);
  myGymEnv->SetFlowMonitor(&flowHelper, monitor);

  

  std::map<uint32_t, ApplicationContainer> appmap;
  std::map<uint32_t, ApplicationContainer> servermap;
  std::map<uint32_t, uint32_t> flowmap;
  uint32_t flowid = 1;
  int x_direction[4] = {0,0,1,-1};
  int y_direction[4] = {1,-1,0,0};
  for (int i=0; i<int(xSize); i++) {
    for (int j=0; j<int(ySize); j++){
      NS_LOG_UNCOND ("Executing row" << i << "Column "<< j);
      for (uint32_t k=0; k<4; k++){
        int neighborX = i + x_direction[k];
        int neighborY = j + y_direction[k];
        if (neighborX >= 0 && neighborX < int(xSize) && neighborY >=0 && neighborY < int(ySize)){
          NS_LOG_UNCOND("From ("<<i<<","<<j<<") to ("<<neighborX<<","<<neighborY<<")");
          Time interPacketInterval = Seconds (0.01);
          uint16_t port = 4000 + k;
          UdpServerHelper server (port);
          ApplicationContainer serverapp = server.Install (grid.GetNode(neighborX,neighborY));
          serverapp.Start (Seconds (0.5));
          serverapp.Stop (Seconds (simulationTime));

          UdpClientHelper client(grid.GetIpv4Address(neighborX, neighborY), port);
          client.SetAttribute ("Interval", TimeValue (interPacketInterval));
          client.SetAttribute ("PacketSize", UintegerValue (1024));
          client.SetAttribute ("MaxPackets", UintegerValue(0));
          ApplicationContainer apps;
          apps = client.Install(grid.GetNode(i,j));
          apps.Start (Seconds(0.5));
          apps.Stop(Seconds(simulationTime));
          uint32_t id1 = i*ySize +j;
          uint32_t id2 = neighborX*ySize + neighborY;
          uint32_t key = id1 *(xSize*ySize) + id2;
          appmap[key] = apps;
          servermap[key] = serverapp.Get(0);
          flowmap[key] = flowid;
          flowid++;
        }
      }
    }
  }
  myGymEnv->SetAppMap(appmap);
  myGymEnv->SetServerMap(servermap);
  myGymEnv->SetFlowMap(flowmap);

  time_t tt = time(NULL);
  struct tm* t= localtime(&tt);
  char buf[200];
  sprintf(buf, "%d-%02d-%02d_%02d:%02d:%02d.txt", 
    t->tm_year + 1900,
    t->tm_mon + 1,
    t->tm_mday,
    t->tm_hour,
    t->tm_min,
    t->tm_sec);
  std::ofstream ofs;
  ofs.open(std::string(buf), std::ios::out);
  ofs << "bandwidth:" << link_bandwidth<< std::endl;
  ofs << "delay:" << link_delay << std::endl;
  ofs << "Xsize:" << xSize << std::endl;
  ofs << "Ysize:" << ySize << std::endl;
  ofs.close();
  myGymEnv->SetFn(std::string(buf));

  NS_LOG_UNCOND ("Simulation start");
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  NS_LOG_UNCOND ("Simulation stop");
  
  openGym->NotifySimulationEnd();
  Simulator::Destroy ();

}
