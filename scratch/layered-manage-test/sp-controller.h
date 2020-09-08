/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Campinas (Unicamp)
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
 * Author:  Luciano Chaves <luciano@lrc.ic.unicamp.br>
 */

#ifndef SP_CONTROLLER_H
#define SP_CONTROLLER_H

#include <ns3/ofswitch13-module.h>
#include <ns3/network-module.h>
#include <ns3/internet-module.h>
#include <ns3/ofswitch13-controller.h>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
#include <sys/time.h> 
#include <iomanip>
using namespace ns3;
using namespace std;

typedef uint64_t dpid_t;

typedef struct NodeInfo
{
    int plane;
    int index;
} nodeInfo_t;



/**
 * \brief An border OpenFlow 1.3 controller
 */
class SPController : public OFSwitch13Controller
{
public:
  SPController ();          //!< Default constructor.
  virtual ~SPController (); //!< Dummy destructor.

  /** Destructor implementation */
  virtual void DoDispose ();

  /**
   * Register this type.
   * \return The object TypeId.
   */
  static TypeId GetTypeId (void);

  /**
   * Handle a packet in message sent by the switch to this controller.
   * \note Inherited from OFSwitch13Controller.
   * \param msg The OpenFlow received message.
   * \param swtch The remote switch metadata.
   * \param xid The transaction id from the request message.
   * \return 0 if everything's ok, otherwise an error number.
   */
  ofl_err HandlePacketIn (
    struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);

  //void ImportOutportMap(map<int,map<int,NetDeviceContainer>> swOutPortMap);
  void ImportNodes(NodeContainer switches);
  void ImportServers(NodeContainer servers);
 
  void ImportBasicInfo(int plane, int index);
  void ImportDpidInfo(nodeInfo_t* dpidInfo, uint16_t len);
  void ImportDpidPortMap(int** dpidPortMap, uint16_t len);
  void ImportDpidAdj(double** dpidAdj, uint16_t len);

  void ImportFlag(int flag);
  void ImportDomainConfig(int cnum, int scnum, map<int,int>s2c, map<int,int>c2sc);
  void ImportCLocation(int time, int cseq, uint64_t dpid);
  void ImportSCLocation(int time, int scseq, uint64_t dpid);
  void IsSC(bool issc); 
  void deleteEntry(dpid_t dp, Mac48Address src48, Mac48Address dst48, int sdomainId);
  

  int m_flag;
  bool m_issc;
  int m_scnum;
  int m_cnum;
  map<int, int> m_s2c;
  map<int, int> m_c2sc;
  map<int, uint64_t> c_loc;
  map<int, uint64_t> sc_loc;
  int reqcnt;
  long long qdelay;
  double pdelay;

  typedef struct PktInCtx
  {
      struct ofl_msg_packet_in *msg;
      Ptr<const RemoteSwitch> swtch;
      uint32_t xid;
      struct timeval t_start;
      double platency;
  } pktInCtx_t;

  map<int, queue<pktInCtx_t>* > ctrlq;
  map<int, queue<pktInCtx_t>* > sctrlq;

  map<int, uint64_t> t_cloc[1000];
  map<int, uint64_t> t_scloc[1000];

  void HandlePacketInHelper(queue<pktInCtx_t>* qe);
   
  //void HandlePacketInHelper(int* a);

  dpid_t getDpidByIndex(int i);
 
     
  
  /**
   * Save the pair IP / MAC address in ARP table.
   * \param ipAddr The IPv4 address.
   * \param macAddr The MAC address.
   */
  void SaveArpEntry (Ipv4Address ipAddr, Mac48Address macAddr);

  void SaveMac2Dpid (Mac48Address macAddr, uint64_t dpid);

  Ptr<Node> FindNodeByDpid(uint64_t dpid, int& index);
  NodeContainer m_switches;
  NodeContainer m_servers;

  

  void PathHelper(dpid_t srcDpid, dpid_t dstDpid, bool right, bool up, double cur_delay, vector<dpid_t>& cur_path, double min_delay, vector<dpid_t>& m_path);
  void calPath(dpid_t srcDpid, dpid_t dstDpid, vector<dpid_t>& path);

  uint32_t updateL2Table(vector<dpid_t> path, Mac48Address dst48);
  void insertFlowTable(vector<dpid_t> path, Mac48Address dst48);
  void insertCrossDomainFlowTable(vector<dpid_t> path, Mac48Address src48, Mac48Address dst48, int sdomainId);
  void insertDomainFlowTable(vector<dpid_t> path, Mac48Address src48, Mac48Address dst48, int domainId);
  bool turnRight(int src_plane, int dst_plane);
  bool turnUp(int src_index, int dst_index);
  
  //void replyPacketOut(vector<dpid_t> path); 

protected:
  // Inherited from OFSwitch13Controller
  void HandshakeSuccessful (Ptr<const RemoteSwitch> swtch);

private:
  
  uint16_t m_dpidlen;
  nodeInfo_t* m_dpidInfo;
  int** m_dpidPortMap;
  double** m_dpidAdj;
  int m_plane;
  int m_index;
  bool* visited;
  dpid_t* last_hop;
  double* weight;
  double* cost;  
  vector<int> crflag[128][128];
  vector<int> rflag[128][128];

  



  typedef std::map<Mac48Address, uint32_t> L2Table_t;

  typedef std::map<uint64_t, L2Table_t> DatapathMap_t;
  
  DatapathMap_t m_learnedInfo;

  void TopologyConstruction( void );

  void ConfigureSwitch (Ptr<const RemoteSwitch> swtch);

  /**
   * Handle TCP connection request
   * \param msg The packet-in message.
   * \param swtch The switch information.
   * \param xid Transaction id.
   * \return 0 if everything's ok, otherwise an error number.
   */
  ofl_err HandleConnectionRequest (
    struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);

  /**
   * Extract an IPv4 address from packet match.
   * \param oxm_of The OXM_IF_* IPv4 field.
   * \param match The ofl_match structure pointer.
   * \return The IPv4 address.
   */
  Ipv4Address ExtractIpv4Address (uint32_t oxm_of, struct ofl_match* match);


  /**
   * Perform an ARP resolution
   * \param ip The Ipv4Address to search.
   * \return The MAC address for this ip.
   */
  Mac48Address GetArpEntry (Ipv4Address ip);

  /** Map saving <IPv4 address / MAC address> */
  typedef std::map<Ipv4Address, Mac48Address> IpMacMap_t;
  typedef std::map<Mac48Address, uint64_t> MacDpidMap_t;
  IpMacMap_t m_arpTable;          //!< ARP resolution table.
  MacDpidMap_t m_macDpidTable;

  ofl_err HandleArpPacketIn (struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch, uint32_t xid);
  ofl_err HandlePacketInBackup (struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch, uint32_t xid);
  Ptr<Packet> CreateArpReply (Mac48Address srcMac, Ipv4Address srcIp, Mac48Address dstMac, Ipv4Address dstIp);
};

#endif /* QOS_CONTROLLER_H */
