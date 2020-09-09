#include "basic.h"




string 
double2string(double a){
  stringstream strStream;  
  strStream << a; 
  string s = strStream.str();  
  return s;
}
 
//datarate is Mbps
/*
void
CreateUdpApp(int src, int dst, float datarate, Ipv4InterfaceContainer serverIpIfaces, NodeContainer hosts)
{
    Address remoteServerAddr = Address(serverIpIfaces.GetAddress(dst));
    std::cout<<"remote addr:" << remoteServerAddr<<std::endl;
    UdpClientHelper helper(remoteServerAddr, 11399);
    helper.SetAttribute("Interval", TimeValue(MilliSeconds(1/datarate)));
    NodeContainer clients (hosts.Get(src));
    ApplicationContainer curapp = helper.Install (clients);
    apps.Add(curapp.Get(0));
}
*/
 
void trafficgen(int src, int dst, uint16_t port, double stime, double etime, Ipv4InterfaceContainer serverIpIfaces, NodeContainer hosts){

   cout<<"etime="<<etime<<endl;
   UdpServerHelper server (port);
   ApplicationContainer apps = server.Install (hosts.Get (dst));
   apps.Start (Seconds (stime));
   apps.Stop (Seconds (etime));
 //
 // Create one UdpClient application to send UDP datagrams from node zero to
 // node one.
 //
   const uint32_t MaxPacketSize = 1024;   //bytes
   Time interPacketInterval = Seconds (0.1);
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


