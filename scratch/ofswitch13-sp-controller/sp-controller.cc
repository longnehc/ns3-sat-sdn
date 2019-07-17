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

#include "sp-controller.h"
#include <ns3/network-module.h>
#include <ns3/internet-module.h>

#include <boost/graph/adjacency_list.hpp>
using namespace boost;
typedef boost::adjacency_list<listS, vecS, undirectedS> mygraph;

NS_LOG_COMPONENT_DEFINE ("SPController");
NS_OBJECT_ENSURE_REGISTERED (SPController);

SPController::SPController ()
{
  NS_LOG_FUNCTION (this);

  TopologyConstruction();
}

SPController::~SPController ()
{
  NS_LOG_FUNCTION (this);
}

void
SPController::DoDispose ()
{
  NS_LOG_FUNCTION (this);

  OFSwitch13Controller::DoDispose ();
}

TypeId
SPController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SPController")
    .SetParent<OFSwitch13Controller> ()
    .SetGroupName ("OFSwitch13")
    .AddConstructor<SPController> ()
  ;
  return tid;
}

void
SPController::ImportServers(NodeContainer servers)
{
    NS_LOG_FUNCTION(this);
    m_servers = servers;
}

void
SPController::ImportNodes(NodeContainer switches)
{
    NS_LOG_FUNCTION(this);
    
    m_switches = switches;
/*    
    NodeContainer::Iterator i;
    for (i = switches.Begin (); i != switches.End (); ++i)
    {
        Ptr<OFSwitch13Device> ofdev = (*i)->GetObject<OFSwitch13Device>();
        uint64_t dpid = ofdev->GetDpId();
        

        Ptr<Ipv4> ipv4 = (*i)->GetObject<Ipv4>();
        for(uint32_t ifNum=0; ifNum<ipv4->GetNInterfaces(); ifNum++)
        {
            Ptr< NetDevice > dev = ipv4->GetNetDevice(ifNum);
            Mac48Address macaddr = Mac48Address::ConvertFrom(dev->GetAddress());
            for(uint32_t addrNum = 0; addrNum < ipv4->GetNAddresses(ifNum); addrNum++)
            {
                Ipv4InterfaceAddress address = ipv4->GetAddress(ifNum, addrNum);
                Ipv4Address ipaddr = address.GetLocal();
                
                std::ostringstream cmd;
                cmd << "flow-mod cmd=add,table=0,prio=100 eth_dst="
                    <<macaddr << " apply:output=local";
                
                //std::cout<<DpctlExecute (dpid, cmd.str ())<<"!"<<std::endl;
                if(ipaddr != Ipv4Address("127.0.0.1"))
                {
                    SaveArpEntry (ipaddr, macaddr);
                }
            }
        }
    }
*/
}

Ptr<Node>
SPController::FindNodeByDpid(uint64_t dpid, int& index)
{
    NodeContainer::Iterator i;
    int count = 0;
    for (i = m_switches.Begin (); i != m_switches.End (); ++i)
    {
        Ptr<OFSwitch13Device> ofdev = (*i)->GetObject<OFSwitch13Device>();
        uint64_t id = ofdev->GetDpId();
        if (id == dpid ){
            index = count;
            return (*i);
        }
        count++;
    }
    index = -1;
    return NULL;
}

/*
void
SPController::AddTCPFlowEntry(Ptr<const RemoteSwitch> swtch, Ipv4Addr ipSrc, 
        Ipv4Addr ipDst, TcpAddr tcpSrc, TCPAddr tcpDst, uint32_t outPort)
{
  NS_LOG_FUNCTION (thjs << swtch << ipSrc << ipDst << outPort);

  std::ostringstream cmd;
  cmd << "flow-mod cmd=add,table=0,idle=5,flags=0x0001"
      << ",prio=100 ip_src=" << ipSrc 
      << " ip_dst=" << ipDst
      << " tcp_src=" << tcpSrc
      << " tcp_dst=" << tcpDst
      << " apply:output=" << outPort;
  DpctlExecute (swtch, cmd.str ());
}

void
SPController::AddDefaultFlowEntry(Ptr<const RemoteSwitch> swtch, Mac48Address src, uint32_t outPort)
{
  NS_LOG_FUNCTION (thjs << swtch << src << outPort);
  std::ostringstream cmd;
  cmd << "flow-mod cmd=add,table=0,idle=5,flags=0x0001"
      << ",prio=10 eth_dst=" << src
      << " apply:output=" << outPort;
  DpctlExecute (swtch, cmd.str ());
}
*/

ofl_err
SPController::HandlePacketIn (
  struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);

  char *msgStr =
    ofl_structs_match_to_string ((struct ofl_match_header*)msg->match, 0);
  NS_LOG_DEBUG ("Packet in match: " << msgStr);
  free (msgStr);

  uint16_t ethType;
  struct ofl_match_tlv *tlv;
  tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
  memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));
  if (msg->reason == OFPR_ACTION)
  {
      
      //calculate shortest path
      if (ethType == ArpL3Protocol::PROT_NUMBER)
      {
          NS_LOG_FUNCTION("is arp");     
          return HandleArpPacketIn (msg, swtch, xid);
      }
  }
  else if(msg->reason == OFPR_NO_MATCH)
  {
    NS_LOG_FUNCTION("cannot handle packet in");
    if (ethType == Ipv4L3Protocol::PROT_NUMBER)
        NS_LOG_FUNCTION("is udp");     
    
    Mac48Address dst48;
    struct ofl_match_tlv *ethDst =
      oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->match);
    dst48.CopyFrom (ethDst->value);
    
    uint16_t ethType;
    tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
    memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));
    //populate routing table
    //AddDefaultFlowEntry();
  }
  // All handlers must free the message when everything is ok
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}

void
SPController::HandshakeSuccessful (Ptr<const RemoteSwitch> swtch)
{
  NS_LOG_FUNCTION (this << swtch);

  // This function is called after a successfully handshake between controller
  // and each switch. Let's check the switch for proper configuration.

  DpctlExecute (swtch, "flow-mod cmd=add,table=0,prio=0 "
                "apply:output=ctrl:128");

  DpctlExecute (swtch, "set-config miss=128");
  
  // Redirect ARP requests to the controller
  DpctlExecute (swtch, "flow-mod cmd=add,table=0,prio=20 "
                "eth_type=0x0806,arp_op=1 apply:output=ctrl");
 
  NS_LOG_FUNCTION(swtch->GetIpv4());
  // Recieve packet 
  //DpctlExecute (swtch, "flow-mod cmd=add,table=0,prio=20 "
  //              " apply:output=local");
  

  int index = 0;
  Ptr<Node> node = FindNodeByDpid(swtch->GetDpId(), index);
  Ptr<OFSwitch13Device> ofdev = node->GetObject<OFSwitch13Device>();
  NS_LOG_FUNCTION(ofdev->GetNSwitchPorts());
  Ptr<Ipv4> ipv4 = m_servers.Get(index)->GetObject<Ipv4>();
  for(uint32_t ifNum=0; ifNum<ipv4->GetNInterfaces(); ifNum++)
  {
      Ptr< NetDevice > dev = ipv4->GetNetDevice(ifNum);
      Mac48Address macaddr = Mac48Address::ConvertFrom(dev->GetAddress());
      for(uint32_t addrNum = 0; addrNum < ipv4->GetNAddresses(ifNum); addrNum++)
      {
          Ipv4InterfaceAddress address = ipv4->GetAddress(ifNum, addrNum);
          Ipv4Address ipaddr = address.GetLocal();
          
          std::ostringstream cmd;
          cmd << "flow-mod cmd=add,table=0,prio=100"
              << " eth_dst=" << macaddr 
              << " apply:output=" << 0;
          
          std::cout<<DpctlExecute (swtch, cmd.str ())<<"!"<<std::endl;
          if(ipaddr != Ipv4Address("127.0.0.1"))
          {
              SaveArpEntry (ipaddr, macaddr);
          }
      }
  }
}


void 
SPController::TopologyConstruction()
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule (MilliSeconds (1000), &SPController::TopologyConstruction, this);
}

ofl_err
SPController::HandleArpPacketIn (
  struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);

  struct ofl_match_tlv *tlv;

  // Get ARP operation
  uint16_t arpOp;
  tlv = oxm_match_lookup (OXM_OF_ARP_OP, (struct ofl_match*)msg->match);
  memcpy (&arpOp, tlv->value, OXM_LENGTH (OXM_OF_ARP_OP));

  // Get input port
  uint32_t inPort;
  tlv = oxm_match_lookup (OXM_OF_IN_PORT, (struct ofl_match*)msg->match);
  memcpy (&inPort, tlv->value, OXM_LENGTH (OXM_OF_IN_PORT));

  // Get source and target IP address
  Ipv4Address srcIp, dstIp;
  srcIp = ExtractIpv4Address (OXM_OF_ARP_SPA, (struct ofl_match*)msg->match);
  dstIp = ExtractIpv4Address (OXM_OF_ARP_TPA, (struct ofl_match*)msg->match);

  // Get Source MAC address
  Mac48Address srcMac, dstMac;
  tlv = oxm_match_lookup (OXM_OF_ARP_SHA, (struct ofl_match*)msg->match);
  srcMac.CopyFrom (tlv->value);
  tlv = oxm_match_lookup (OXM_OF_ARP_THA, (struct ofl_match*)msg->match);
  dstMac.CopyFrom (tlv->value);

  // Check for ARP request
  if (arpOp == ArpHeader::ARP_TYPE_REQUEST)
    {
      uint8_t replyData[64];

      // Check for existing information
      Mac48Address replyMac = GetArpEntry (dstIp);
      Ptr<Packet> pkt = CreateArpReply (replyMac, dstIp, srcMac, srcIp);
      NS_ASSERT_MSG (pkt->GetSize () == 64, "Invalid packet size.");
      pkt->CopyData (replyData, 64);

      // Send the ARP replay back to the input port
      struct ofl_action_output *action =
        (struct ofl_action_output*)xmalloc (sizeof (struct ofl_action_output));
      action->header.type = OFPAT_OUTPUT;
      action->port = OFPP_IN_PORT;
      action->max_len = 0;

      // Send the ARP reply within an OpenFlow PacketOut message
      struct ofl_msg_packet_out reply;
      reply.header.type = OFPT_PACKET_OUT;
      reply.buffer_id = OFP_NO_BUFFER;
      reply.in_port = inPort;
      reply.data_length = 64;
      reply.data = &replyData[0];
      reply.actions_num = 1;
      reply.actions = (struct ofl_action_header**)&action;

      SendToSwitch (swtch, (struct ofl_msg_header*)&reply, xid);
      free (action);
    }
    else
    {
        NS_LOG_FUNCTION("error!");
    }
  // All handlers must free the message when everything is ok
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}

Ptr<Packet>
SPController::CreateArpReply (Mac48Address srcMac, Ipv4Address srcIp,
                               Mac48Address dstMac, Ipv4Address dstIp)
{
  NS_LOG_FUNCTION (this << srcMac << srcIp << dstMac << dstIp);

  Ptr<Packet> packet = Create<Packet> ();

  // ARP header
  ArpHeader arp;
  arp.SetReply (srcMac, srcIp, dstMac, dstIp);
  packet->AddHeader (arp);

  // Ethernet header
  EthernetHeader eth (false);
  eth.SetSource (srcMac);
  eth.SetDestination (dstMac);
  if (packet->GetSize () < 46)
    {
      uint8_t buffer[46];
      memset (buffer, 0, 46);
      Ptr<Packet> padd = Create<Packet> (buffer, 46 - packet->GetSize ());
      packet->AddAtEnd (padd);
    }
  eth.SetLengthType (ArpL3Protocol::PROT_NUMBER);
  packet->AddHeader (eth);

  // Ethernet trailer
  EthernetTrailer trailer;
  if (Node::ChecksumEnabled ())
    {
      trailer.EnableFcs (true);
    }
  trailer.CalcFcs (packet);
  packet->AddTrailer (trailer);

  return packet;
}

Mac48Address
SPController::GetArpEntry (Ipv4Address ip)
{
  IpMacMap_t::iterator ret;
  ret = m_arpTable.find (ip);
  if (ret != m_arpTable.end ())
    {
      NS_LOG_INFO ("Found ARP entry: " << ip << " - " << ret->second);
      return ret->second;
    }
  NS_ABORT_MSG ("No ARP information for this IP.");
}

void
SPController::SaveArpEntry (Ipv4Address ipAddr, Mac48Address macAddr)
{
  NS_LOG_FUNCTION(this<<ipAddr<<macAddr);
  std::pair<Ipv4Address, Mac48Address> entry (ipAddr, macAddr);
  std::pair <IpMacMap_t::iterator, bool> ret;
  ret = m_arpTable.insert (entry);
  if (ret.second == true)
    {
      NS_LOG_INFO ("New ARP entry: " << ipAddr << " - " << macAddr);
      return;
    }
}

Ipv4Address
SPController::ExtractIpv4Address (uint32_t oxm_of, struct ofl_match* match)
{
  switch (oxm_of)
    {
    case OXM_OF_ARP_SPA:
    case OXM_OF_ARP_TPA:
    case OXM_OF_IPV4_DST:
    case OXM_OF_IPV4_SRC:
      {
        uint32_t ip;
        int size = OXM_LENGTH (oxm_of);
        struct ofl_match_tlv *tlv = oxm_match_lookup (oxm_of, match);
        memcpy (&ip, tlv->value, size);
        return Ipv4Address (ntohl (ip));
      }
    default:
      NS_ABORT_MSG ("Invalid IP field.");
    }
}
