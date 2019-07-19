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
#include <iostream>
#include <vector>
#include <map>
using namespace ns3;
using namespace std;

typedef uint64_t dpid_t;


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
  void ImportDpidAdj(map<dpid_t, vector<dpid_t>> dpidAdj);
  void ImportDpidPortMap(map<dpid_t, map<dpid_t, uint32_t>> dpidPortMap);
     
  
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
  map<int,map<int,NetDeviceContainer>> m_outPortMap;
  map<dpid_t, map<dpid_t, uint32_t>> m_dpidPortMap;

  void calPath(dpid_t srcDpid, dpid_t dstDpid, vector<dpid_t>& path);

  uint32_t updateL2Table(vector<dpid_t> path, Mac48Address dst48);

  //void replyPacketOut(vector<dpid_t> path); 

protected:
  // Inherited from OFSwitch13Controller
  void HandshakeSuccessful (Ptr<const RemoteSwitch> swtch);

private:
  
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
  map<dpid_t, vector<dpid_t>> m_dpidAdj;

  ofl_err HandleArpPacketIn (struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch, uint32_t xid);
  
  Ptr<Packet> CreateArpReply (Mac48Address srcMac, Ipv4Address srcIp, Mac48Address dstMac, Ipv4Address dstIp);
};

#endif /* QOS_CONTROLLER_H */
