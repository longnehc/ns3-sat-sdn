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


#include <boost/graph/adjacency_list.hpp>
#include <algorithm>

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
  delete[] cost;
  delete[] weight;
  delete[] last_hop;
  delete[] visited;  
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
SPController::ImportBasicInfo(int plane, int index)
{
    NS_LOG_FUNCTION(this);
    m_plane = plane;
    m_index = index;
}
void 
SPController::ImportDpidInfo(nodeInfo_t* dpidInfo, uint16_t len)
{
    NS_LOG_FUNCTION(this);
    m_dpidInfo = dpidInfo;
    m_dpidlen = len;
    visited = new bool[m_dpidlen];    //from 1 to m_dpidlen-1
    last_hop = new dpid_t[m_dpidlen];
    weight = new double[m_dpidlen];
    cost = new double[m_dpidlen];
    for(int i = 0; i < 128; i++){
      for(int j = 0; j < 128; j++){
        vector<int> tv;
        crflag[i][j] = tv;
        rflag[i][j] = tv;
      }
    }
}
   

void
SPController::ImportDpidPortMap(int** dpidPortMap, uint16_t len)
{
    NS_LOG_FUNCTION(this);
    m_dpidPortMap = dpidPortMap;
    m_dpidlen = len;
}

void 
SPController::ImportDpidAdj(double** dpidAdj, uint16_t len)
{
    NS_LOG_FUNCTION(this);
    m_dpidAdj = dpidAdj;
    m_dpidlen = len;
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
}

void
SPController::ImportFlag(int flag)
{
    NS_LOG_FUNCTION(this); 
    m_flag = flag;
}

 

void 
SPController::IsSC(bool issc){
  m_issc = issc;
}
 

void 
SPController::HandlePacketInHelper(queue<pktInCtx_t>* qe){
  //cout<<"ddddddddddddddddfffffffffffffff:"<<endl;
  if(!qe->empty()){
    //cout<<"dddddddddddddddd:"<<qe->size()<<endl;
    pktInCtx_t ctx = qe->front();
    qe->pop();
    struct ofl_msg_packet_in *msg;
    Ptr<const RemoteSwitch> swtch;
    uint32_t xid;

    msg = ctx.msg;
    swtch = ctx.swtch;
    xid = ctx.xid;
    NS_LOG_FUNCTION (this << swtch << xid);
    //static int prio = 100;
    char *msgStr =
      ofl_structs_match_to_string ((struct ofl_match_header*)msg->match, 0);
    NS_LOG_DEBUG ("Packet in match: " << msgStr);
    free (msgStr);

    uint64_t dpId = swtch->GetDpId ();

    std::cout<<"The switch with dpid= "<<dpId<<" received packetIn!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"Flag= "<<m_flag<<" !!!!!!!!!!!!!"<<std::endl;

    uint16_t ethType;
    struct ofl_match_tlv *tlv;
    tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
    memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));
    if (msg->reason == OFPR_ACTION)
    {
        if (ethType == ArpL3Protocol::PROT_NUMBER)
        {
            NS_LOG_FUNCTION("is arp");  
            //cout<<"dpId="<<dpId<<endl;    
            HandleArpPacketIn (msg, swtch, xid);
            //return;
        }
    }
    else if(msg->reason == OFPR_NO_MATCH)
    {
      NS_LOG_FUNCTION("cannot handle packet in");
      if (ethType == Ipv4L3Protocol::PROT_NUMBER)
          NS_LOG_FUNCTION("is udp");     
      
      uint32_t inPort;
      //uint32_t outPort = OFPP_FLOOD;
      size_t portLen = OXM_LENGTH (OXM_OF_IN_PORT); // (Always 4 bytes)
      struct ofl_match_tlv *input =
        oxm_match_lookup (OXM_OF_IN_PORT, (struct ofl_match*)msg->match);
      memcpy (&inPort, input->value, portLen);


      Mac48Address src48;
      struct ofl_match_tlv *ethSrc =
        oxm_match_lookup (OXM_OF_ETH_SRC, (struct ofl_match*)msg->match);
      src48.CopyFrom (ethSrc->value);
      
      Mac48Address dst48;
      struct ofl_match_tlv *ethDst =
        oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->match);
      dst48.CopyFrom (ethDst->value);
      
      dpid_t srcDpid = m_macDpidTable[src48];
      //std::cout<<"The src dpid for Mac: "<<src48<<" is "<<srcDpid<<std::endl;

      dpid_t dstDpid = m_macDpidTable[dst48];
      //std::cout<<"The dst dpid for Mac: "<<dst48<<" is "<<dstDpid<<std::endl;

      uint16_t ethType;
      tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
      memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));

      //calculating path by dpid
      vector<dpid_t> path;
      calPath(srcDpid, dstDpid, path);
      //insertFlowTable(path, dst48);
      if(m_issc){
        insertCrossDomainFlowTable(path, dst48, m_c2sc[m_s2c[dpId]]);
        vector<int> tv = crflag[srcDpid][dstDpid];
        tv.push_back(m_c2sc[m_s2c[dpId]]);
           //cout<<"tttttt"<<srcDpid<<","<<dstDpid<<","<<tv.size()<<endl;
        crflag[srcDpid][dstDpid] = tv;
      }
      else {
        insertDomainFlowTable(path, dst48, m_s2c[dpId]);
      }
    }
    // All handlers must free the message when everything is ok
       ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  }
  Simulator::Schedule (MilliSeconds (1), &SPController::HandlePacketInHelper, this, qe);
  //return 0;
} 

 
void 
SPController::ImportDomainConfig(int cnum, int scnum, map<int,int>s2c, map<int,int>c2sc){
  m_cnum = cnum;
  m_scnum = scnum;
  m_s2c = s2c;
  m_c2sc = c2sc;
  for(int i = 0; i < cnum; i++){
    queue<pktInCtx_t>* cq = new queue<pktInCtx_t>;
    ctrlq[i] = cq;
    cout<<"eeeeeeeeeeeeeeee:"<<cq->size()<<endl;
    Simulator::Schedule (MilliSeconds (1), &SPController::HandlePacketInHelper, this, cq);
  }

  for(int i = 0; i < scnum; i++){
    queue<pktInCtx_t>* scq = new queue<pktInCtx_t>;
    sctrlq[i] = scq;
    cout<<"fffffffffffffff:"<<scq->size()<<endl;
    Simulator::Schedule (MilliSeconds (1), &SPController::HandlePacketInHelper, this, scq);
  }
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

int domainId = 0;


ofl_err
SPController::HandlePacketIn (
  struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  uint64_t dpId = swtch->GetDpId ();
  pktInCtx_t qele;
  qele.msg = msg;
  qele.swtch = swtch;
  qele.xid = xid;
  if(m_issc) {
    if(sctrlq.find(m_c2sc[m_s2c[dpId]]) != sctrlq.end()){
      Mac48Address src48;
      struct ofl_match_tlv *ethSrc =
        oxm_match_lookup (OXM_OF_ETH_SRC, (struct ofl_match*)msg->match);
      src48.CopyFrom (ethSrc->value);
      
      Mac48Address dst48;
      struct ofl_match_tlv *ethDst =
        oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->match);
      dst48.CopyFrom (ethDst->value);
      
      dpid_t srcDpid = m_macDpidTable[src48];
      //std::cout<<"The src dpid for Mac: "<<src48<<" is "<<srcDpid<<std::endl;

      dpid_t dstDpid = m_macDpidTable[dst48];
      //std::cout<<"The dst dpid for Mac: "<<dst48<<" is "<<dstDpid<<std::endl;

      vector<int> tv = crflag[srcDpid][dstDpid];

      vector<int>::iterator it = find(tv.begin(), tv.end(), m_c2sc[m_s2c[dpId]]);
      if(tv.end() == it) {
        std::cout<<"Super-controller no.="<<m_c2sc[m_s2c[dpId]]<<" received packetIn!!!!!!!!!!!!!"<<std::endl;
        sctrlq[m_c2sc[m_s2c[dpId]]]->push(qele);
      }
    } else {
      NS_ABORT_MSG ("No map key found.");
    }
  } else {
    if(ctrlq.find(m_s2c[dpId]) != ctrlq.end()){
      std::cout<<"Controller no.="<<m_s2c[dpId]<<" received packetIn!!!!!!!!!!!!!"<<std::endl;
      ctrlq[m_s2c[dpId]]->push(qele);
    } else {
      NS_ABORT_MSG ("No map key found.");
    }
  }
  return 0;
}





ofl_err
//SPController::HandlePacketIn (
SPController::HandlePacketInBackup (
  struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);
  static int prio = 100;
  char *msgStr =
    ofl_structs_match_to_string ((struct ofl_match_header*)msg->match, 0);
  NS_LOG_DEBUG ("Packet in match: " << msgStr);
  free (msgStr);

  uint64_t dpId = swtch->GetDpId ();

  std::cout<<"The switch with dpid= "<<dpId<<" received packetIn!!!!!!!!!!!!!"<<std::endl;
  std::cout<<"Flag= "<<m_flag<<" !!!!!!!!!!!!!"<<std::endl;

  uint16_t ethType;
  struct ofl_match_tlv *tlv;
  tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
  memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));
  if (msg->reason == OFPR_ACTION)
  {
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
    
    uint32_t inPort;
    uint32_t outPort = OFPP_FLOOD;
    size_t portLen = OXM_LENGTH (OXM_OF_IN_PORT); // (Always 4 bytes)
    struct ofl_match_tlv *input =
      oxm_match_lookup (OXM_OF_IN_PORT, (struct ofl_match*)msg->match);
    memcpy (&inPort, input->value, portLen);


    Mac48Address src48;
    struct ofl_match_tlv *ethSrc =
      oxm_match_lookup (OXM_OF_ETH_SRC, (struct ofl_match*)msg->match);
    src48.CopyFrom (ethSrc->value);
    
    Mac48Address dst48;
    struct ofl_match_tlv *ethDst =
      oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->match);
    dst48.CopyFrom (ethDst->value);
    
    dpid_t srcDpid = m_macDpidTable[src48];
    //std::cout<<"The src dpid for Mac: "<<src48<<" is "<<srcDpid<<std::endl;

    dpid_t dstDpid = m_macDpidTable[dst48];
    //std::cout<<"The dst dpid for Mac: "<<dst48<<" is "<<dstDpid<<std::endl;

    uint16_t ethType;
    tlv = oxm_match_lookup (OXM_OF_ETH_TYPE, (struct ofl_match*)msg->match);
    memcpy (&ethType, tlv->value, OXM_LENGTH (OXM_OF_ETH_TYPE));

    // Get L2Table for this datapath
     auto it = m_learnedInfo.find (dpId);
     if (it != m_learnedInfo.end ()){
        std::cout<<"Find L2 table for this datapath id="<<dpId<<endl;
        L2Table_t *l2Table = &it->second;
          // Looking for out port based on dst address (except for broadcast)
          if (!dst48.IsBroadcast ()) {
              auto itDst = l2Table->find (dst48);
              if (itDst != l2Table->end ()) {
                  outPort = itDst->second;
                  std::cout<<"port exists destined to: "<<dst48<<" at "<<dpId<<" is "<< outPort<<std::endl; 
              }
              else{
                 //calculating path by dpid
                 vector<dpid_t> path;
                 calPath(srcDpid, dstDpid, path);
                 outPort = updateL2Table(path, dst48);
                 std::cout<<"port calculated for: "<<dst48<<" is "<< outPort<<std::endl; 
              }

              //insert flow table
              std::ostringstream cmd;
              cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                      << ",prio=" << ++prio << " eth_dst=" << dst48
                      << " apply:output=" << outPort;
  
              std::cout<<"Inserting flow entry to dpid: "<<dpId<<" dst = "<<dst48<<" outPort "<< outPort<<std::endl; 
              int stat = DpctlExecute (swtch, cmd.str ());
              if(stat != 0)
                  std::cout<<"Error accured!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
          }
     }
     else{
         NS_LOG_ERROR ("No L2 table for this datapath id " << dpId);
         std::cout<<"No L2 table for this datapath id " << dpId << std::endl;
     }
  }
  // All handlers must free the message when everything is ok
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}

void SPController::HandshakeSuccessful(Ptr<const RemoteSwitch> swtch)
{
  NS_LOG_FUNCTION (this << swtch);

  // This function is called after a successfully handshake between controller
  // and each switch. Let's check the switch for proper configuration.

  DpctlExecute (swtch, "flow-mod cmd=add,table=0,prio=0 "
                "apply:output=ctrl:128");

  DpctlExecute (swtch, "set-config miss=128");
  
  L2Table_t l2Table;
  uint64_t dpId = swtch->GetDpId ();
  
  std::pair<uint64_t, L2Table_t> entry (dpId, l2Table);
  auto ret = m_learnedInfo.insert (entry);
  if (ret.second == false){
    NS_LOG_ERROR ("Table exists for this datapath.");
  }

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
          DpctlExecute (swtch, cmd.str ());
          //std::cout<<DpctlExecute (swtch, cmd.str ())<<"!"<<std::endl;
          if(ipaddr != Ipv4Address("127.0.0.1"))
          {
              SaveArpEntry (ipaddr, macaddr);
              SaveMac2Dpid (macaddr, swtch->GetDpId());
          }
      }
  }
}


void SPController::TopologyConstruction()
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
  //ofl_msg_free ((struct ofl_msg_header*)msg, 0);
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
  //cout<<ip<<endl;
  if (ret != m_arpTable.end ())
    {
      NS_LOG_INFO ("Found ARP entry: " << ip << " - " << ret->second);
      //cout<<"Found ARP entry: " << ip << " - " << ret->second<<endl; 
      return ret->second;
    }
  NS_ABORT_MSG ("No ARP information for this IP.");
}


void
SPController::SaveMac2Dpid (Mac48Address macAddr, uint64_t dpid)
{
  NS_LOG_FUNCTION(this<<macAddr<<dpid);
  std::pair<Mac48Address, uint64_t> entry (macAddr, dpid);
  std::pair <MacDpidMap_t::iterator, bool> ret;
  ret = m_macDpidTable.insert (entry);
  if (ret.second == true)
    {
    //  std::cout<<"Mac: "<<macAddr<<" is connected to switch with dpid = "<<dpid<<std::endl;
      return;
    }
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

void 
SPController::PathHelper(dpid_t srcDpid, dpid_t dstDpid, bool right, bool up, double cur_delay, vector<dpid_t>& cur_path, double min_delay, vector<dpid_t>& m_path) {
  int sp = m_dpidInfo[srcDpid].plane;
  int sn = m_dpidInfo[srcDpid].index;
  int dp = m_dpidInfo[dstDpid].plane;
  int dn = m_dpidInfo[dstDpid].index;

  if(sp == dp && sn == dn){
       cout<<" Dst reached: ("<<sp<<","<<dp<<")!!!!!!!!!!!!!!"<<endl;
      if(cur_delay < min_delay){
        cout<<" Path updated: "<<min_delay<<">"<<cur_delay<<"!!!!!!!!!!!!!!"<<endl;
        min_delay = cur_delay;
        m_path = cur_path;
      }
  }
  int l_plane = (sp == 0) ? m_plane - 1 : sp - 1;
  int r_plane = (sp == m_plane - 1) ? 0 : sp + 1;
  int u_num = (sn == m_index - 1) ? 0 : sn + 1;
  int d_num = (sn == 0) ? m_index-1 : sn - 1;
  cout<<"PathHelper called: "<<srcDpid<<","<<dstDpid<<endl;
  if(right && sp != dp){
  //if(sp != dp){
      dpid_t next_dpid = getDpidByIndex(r_plane * m_index + sn);
      cout<<"To right: "<<r_plane<<" "<<sn<<" "<<m_dpidAdj[srcDpid][next_dpid]<<endl;
      if(m_dpidAdj[srcDpid][next_dpid] != -1) { 
        cur_delay += m_dpidAdj[srcDpid][next_dpid];
        cur_path.push_back(next_dpid);
        cout<<"Moving right from ("<<m_dpidInfo[srcDpid].plane<<","<<m_dpidInfo[srcDpid].index<<")";
        cout<<" to ("<<m_dpidInfo[next_dpid].plane<<","<<m_dpidInfo[next_dpid].index<<"), next_dpid="<<next_dpid<<endl;
        PathHelper(next_dpid, dstDpid, right, up, cur_delay, cur_path, min_delay, m_path);
        cur_path.pop_back();
        cur_delay -= m_dpidAdj[srcDpid][next_dpid];
      }
      else
          cout<<"Right failed"<<endl;
  }
  if(!right && sp != dp){
  //if(sp != dp){
      dpid_t next_dpid = getDpidByIndex(l_plane * m_index + sn);
      cout<<"To left: "<<l_plane<<" "<<sn<<" "<<m_dpidAdj[srcDpid][next_dpid]<<endl;
      if(m_dpidAdj[srcDpid][next_dpid] != -1){     
      cur_delay += m_dpidAdj[srcDpid][next_dpid];
      cur_path.push_back(next_dpid);
      cout<<"Moving left from ("<<m_dpidInfo[srcDpid].plane<<","<<m_dpidInfo[srcDpid].index<<")";
      cout<<" to ("<<m_dpidInfo[next_dpid].plane<<","<<m_dpidInfo[next_dpid].index<<")"<<endl;
      PathHelper(next_dpid, dstDpid, right, up, cur_delay, cur_path, min_delay, m_path);
      cur_path.pop_back();
      cur_delay -= m_dpidAdj[srcDpid][next_dpid];
      }
      else
          cout<<"Left failed"<<endl;
  }
  if(up && sn != dn){
      dpid_t next_dpid = getDpidByIndex(sp * m_index + u_num);
      cout<<"To up: "<<sp<<" "<<u_num<<" "<<m_dpidAdj[srcDpid][next_dpid]<<endl;
      if(m_dpidAdj[srcDpid][next_dpid] != -1){  
        cout<<"Next_dpid in up: "<<next_dpid<<endl;
        cur_delay += m_dpidAdj[srcDpid][next_dpid];
        cur_path.push_back(next_dpid);
        cout<<"Moving up from ("<<m_dpidInfo[srcDpid].plane<<","<<m_dpidInfo[srcDpid].index<<")";
        cout<<" to ("<<m_dpidInfo[next_dpid].plane<<","<<m_dpidInfo[next_dpid].index<<")"<<endl;
        PathHelper(next_dpid, dstDpid, right, up, cur_delay, cur_path, min_delay, m_path);
        cur_path.pop_back();
        cur_delay -= m_dpidAdj[srcDpid][next_dpid];
      }
      else
          cout<<"Up failed"<<endl;
  }
  if(!up && sn != dn) {
      dpid_t next_dpid = getDpidByIndex(sp * m_index + d_num);
      cout<<"To down: "<<sp<<" "<<d_num<<" "<<m_dpidAdj[srcDpid][next_dpid]<<endl;
      if(m_dpidAdj[srcDpid][next_dpid] != -1){  
        cout<<"Next_dpid in down: "<<next_dpid<<endl;
        cur_delay += m_dpidAdj[srcDpid][next_dpid];
        cur_path.push_back(next_dpid);
        cout<<"Moving down from ("<<m_dpidInfo[srcDpid].plane<<","<<m_dpidInfo[srcDpid].index<<")";
        cout<<" to ("<<m_dpidInfo[next_dpid].plane<<","<<m_dpidInfo[next_dpid].index<<")"<<endl;
        PathHelper(next_dpid, dstDpid, right, up, cur_delay, cur_path, min_delay, m_path);
        cur_path.pop_back();
        cur_delay -= m_dpidAdj[srcDpid][next_dpid];
      }
      else
          cout<<"Down failed"<<endl;
  }
}

void
SPController::calPath(dpid_t srcDpid, dpid_t dstDpid, vector<dpid_t>& path) { 
  

  //cout<<"Finding path from srcDpid: "<<srcDpid<<"("<<m_dpidInfo[srcDpid].plane<<","<<m_dpidInfo[srcDpid].index<<")";
  //cout<<" to dstDpid: "<<dstDpid<<"("<<m_dpidInfo[dstDpid].plane<<","<<m_dpidInfo[dstDpid].index<<")"<<endl;;

  double DMAX = 99999999;
  //init
  for(uint32_t i = 1; i < m_dpidlen; i++){
      if(i == srcDpid) {
        visited[i] = true;
        cost[i] = 0;
      }
      else {
        visited[i] = false;
        cost[i] = DMAX;
      }
      if(m_dpidAdj[srcDpid][i] != -1) {
        weight[i] = m_dpidAdj[srcDpid][i];
        //cout<<"The last hop of "<<i<<" is "<<srcDpid<<endl;
        last_hop[i] = srcDpid;
      }
      else {
        weight[i] = DMAX;
        last_hop[i] = 0;
      }
  } 
  for(uint32_t i = 1; i < m_dpidlen; i++){
      int the_node = -1;
      int dmin = DMAX;
      for(uint32_t j = 1; j < m_dpidlen; j++){
          if(j != srcDpid && !visited[j] && weight[j] < dmin){
            dmin = weight[j];
            the_node = j;
          }
      }
      if(the_node == -1) continue;
      //cout<<"Next hop (dpid) = " <<the_node<<endl;
      visited[the_node] = true;
      cost[the_node] = dmin;
      for(uint32_t k = 1; k < m_dpidlen; k++){
          if(!visited[k] && m_dpidAdj[the_node][k] != -1 && weight[the_node] + m_dpidAdj[the_node][k] < weight[k]){
            //cout<<"Node with dpid: "<<k<<" is updated "<<endl;
            weight[k] = weight[the_node] + m_dpidAdj[the_node][k];
            last_hop[k] = the_node;
          }
      }
  }

  dpid_t cur = dstDpid;
  do{
    path.push_back(cur);
    cur = last_hop[cur];
  } while(cur!= srcDpid);
  path.push_back(srcDpid);
  reverse(path.begin(), path.end());
  
  //for(uint32_t i = 0; i < path.size(); i++)
  //  cout<<path[i]<<"->";
  //cout<<endl;

 
 /*
  
  */
  //bool right = turnRight(m_dpidInfo[srcDpid].plane, m_dpidInfo[dstDpid].plane);
  //bool up = turnUp(m_dpidInfo[srcDpid].index, m_dpidInfo[dstDpid].index);
 // double cur_delay = 0;
 // double min_delay = 9999999;
 // vector<dpid_t> cur_path;
 // cur_path.push_back(srcDpid);
 // PathHelper(srcDpid, dstDpid, right, up, cur_delay, cur_path, min_delay, path);
/*
  path.push_back(srcDpid);
  std::cout<<"Path to "<<dstDpid<<" next hop: "<<srcDpid<<std::endl;
  if(srcDpid == dstDpid) return;
  
  for(uint32_t i = 0; i < m_dpidAdj[srcDpid].size(); i++) {
    bool loop = false;
    for(uint32_t j = 0; j < path.size(); j++) {
        if(m_dpidAdj[srcDpid][i] == path[j])
            loop = true;
    }
    if(!loop)
      calPath(m_dpidAdj[srcDpid][i], dstDpid, path);
  }
*/
}

void 
SPController::insertCrossDomainFlowTable(vector<dpid_t> path, Mac48Address dst48, int sdomainId){
  int prio = 100;
  for(uint32_t i = 0; i < path.size() - 1; i++) {
    /*
    cout<<"m_s2c[path[i]] : "<<m_s2c[path[i]]<<endl;
    cout<<"m_s2c[path[i+1]] : "<<m_s2c[path[i+1]]<<endl;
    cout<<"m_c2sc[m_s2c[path[i]]]: "<<m_c2sc[m_s2c[path[i]]]<<endl;
    cout<<"m_c2sc[m_s2c[path[i+1]]]: "<<m_c2sc[m_s2c[path[i+1]]]<<endl;
    cout<<"sdomainId:"<<sdomainId<<endl;
    */
    if(m_dpidPortMap[path[i]][path[i+1]] != -1 && m_s2c[path[i]] != m_s2c[path[i+1]] 
      && m_c2sc[m_s2c[path[i]]] == sdomainId && m_c2sc[m_s2c[path[i+1]]] == sdomainId){
        uint32_t outPort;
        outPort = m_dpidPortMap[path[i]][path[i+1]];
        cout<<"Inserting cross-domain flowtable of dpid = "<<path[i]<<", ";
        cout<<"the port to "<<dst48<<" is "<<outPort<<endl;
        cout<<"Superdomain="<<sdomainId<<", domain1="<<m_s2c[path[i]]<<",domain2="<<m_s2c[path[i+1]]<<endl;


        ostringstream cmd;
        cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                      << ",prio=" << ++prio << " eth_dst=" << dst48
                      << " apply:output=" << outPort;
        int stat = DpctlExecute (path[i], cmd.str ());
        if(stat != 0)
          NS_ABORT_MSG("Error accured!!!!!!!!!!!!!!!!!!!!!");    
      }
      else {
        //cout<<"The port from dpid="<<path[i]<<" to dpid="<<path[i+1]<<" doesn't exists"<<endl;
        //NS_ABORT_MSG ("No next hop available of "<<path[i]);
      }
  }
}


void 
SPController::insertDomainFlowTable(vector<dpid_t> path, Mac48Address dst48, int domainId){
  int prio = 100;
  for(uint32_t i = 0; i < path.size() - 1; i++) {
    if(m_dpidPortMap[path[i]][path[i+1]] != -1 && m_s2c[path[i]] == domainId 
      && m_s2c[path[i+1]] == domainId){
        uint32_t outPort;
        outPort = m_dpidPortMap[path[i]][path[i+1]];
        cout<<"Inserting domain flowtable of dpid = "<<path[i]<<", ";
        cout<<"the port to "<<dst48<<" is "<<outPort<<", domain="<<domainId<<endl;
        ostringstream cmd;
        cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                      << ",prio=" << ++prio << " eth_dst=" << dst48
                      << " apply:output=" << outPort;
        int stat = DpctlExecute (path[i], cmd.str ());
        if(stat != 0)
          NS_ABORT_MSG("Error accured!!!!!!!!!!!!!!!!!!!!!");    
      }
      /*
      else {
        cout<<"The port from dpid="<<path[i]<<" to dpid="<<path[i+1]<<" doesn't exists"<<endl;
        NS_ABORT_MSG ("No next hop available of "<<path[i]);
      }*/
  }
  if(m_s2c[path[path.size()-1]] == domainId){
     uint32_t outPort = 1;
     cout<<"Inserting domain flowtable of dpid = "<<path[path.size()-1]<<", ";
          cout<<"The port to "<<dst48<<" is "<<outPort<<", domain="<<domainId<<endl;
     //insert flow table
     ostringstream cmd;
     cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                        << ",prio=" << ++prio << " eth_dst=" << dst48
                        << " apply:output=" << outPort;
      int stat = DpctlExecute (path[path.size()-1], cmd.str ());
      if(stat != 0)
            NS_ABORT_MSG("Error accured!!!!!!!!!!!!!!!!!!!!!");   
  }
}

void 
SPController::insertFlowTable(vector<dpid_t> path, Mac48Address dst48){
  //map<dpid_t, uint32_t> dpid2port;
  static int prio = 100;
  for(uint32_t i = 0; i < path.size() - 1; i++) {
    if(m_dpidPortMap[path[i]][path[i+1]] != -1){
        uint32_t outPort;
        outPort = m_dpidPortMap[path[i]][path[i+1]];
        cout<<"Inserting flowtable of dpid = "<<path[i]<<", ";
        cout<<"the port to "<<dst48<<" is "<<outPort<<endl;
        //dpid2port[path[i]] = outPort;
        ostringstream cmd;
        cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                      << ",prio=" << ++prio << " eth_dst=" << dst48
                      << " apply:output=" << outPort;
        int stat = DpctlExecute (path[i], cmd.str ());
        if(stat != 0)
          NS_ABORT_MSG("Error accured!!!!!!!!!!!!!!!!!!!!!");    
      }
      else {
        cout<<"The port from dpid="<<path[i]<<" to dpid="<<path[i+1]<<" doesn't exists"<<endl;
        NS_ABORT_MSG ("No next hop available of "<<path[i]);
      }
  }

   uint32_t outPort = 1;
   cout<<"Inserting flowtable of dpid = "<<path[path.size()-1]<<", ";
        cout<<"The port to "<<dst48<<" is "<<outPort<<endl;
   //insert flow table
   ostringstream cmd;
   cmd << "flow-mod cmd=add,table=0,flags=0x0001"
                      << ",prio=" << ++prio << " eth_dst=" << dst48
                      << " apply:output=" << outPort;
    int stat = DpctlExecute (path[path.size()-1], cmd.str ());
    if(stat != 0)
          NS_ABORT_MSG("Error accured!!!!!!!!!!!!!!!!!!!!!");   
}

uint32_t
SPController::updateL2Table(vector<dpid_t> path, Mac48Address dst48){
  uint32_t ret;
  L2Table_t *l2Table;
  uint32_t i = 0;
  for(; i < path.size() - 1; i++) {
    auto it = m_learnedInfo.find(path[i]);
    uint32_t outPort;
    if(it != m_learnedInfo.end()){
      l2Table = &it->second;
      if(m_dpidPortMap[path[i]][path[i+1]] != -1){
        outPort = m_dpidPortMap[path[i]][path[i+1]];
        if(i == 0) ret = outPort;         //return the port number of the next hop
      }
      else {
        cout<<"The port from dpid: "<<path[i]<<" to depid "<<path[i+1]<<" doesn't exists"<<endl;
        ret = -1;
        NS_LOG_ERROR ("No next hop available of "<<path[i]);
      }
    }
    else {
      NS_LOG_ERROR ("No L2 table for this datapath id " << path[i]);
    }
    std::cout<<"Updating L2Table of dpid = "<<path[i]<<std::endl;
    std::cout<<"The port to "<<dst48<<" is "<<outPort<<std::endl;
    std::pair<Mac48Address, uint32_t> entry (dst48, outPort);
    //insert into table
    auto insrt_ret = l2Table->insert(entry);
    if(insrt_ret.second == false) {
      NS_LOG_ERROR ("Can't insert mac48address / port pair");
    }
  }
  //The outPort of the last hop is 1
  std::cout<<"Updating L2Table of dpid = "<<path[i]<<std::endl;
  std::cout<<"The port to "<<dst48<<" is "<<1<<std::endl;
  auto it = m_learnedInfo.find(path[i]);
  l2Table = &it->second;
  std::pair<Mac48Address, uint32_t> final_entry (dst48, 1);
  auto insrt_ret = l2Table->insert(final_entry);
  if(insrt_ret.second == false) {
      NS_LOG_ERROR ("Can't insert mac48address / port pair");
  }
  return ret;
}


bool 
SPController::turnRight(int src_plane, int dst_plane){
    if(src_plane < dst_plane && dst_plane - src_plane <= m_plane/2)
      return true;
    else if(src_plane > dst_plane && src_plane - dst_plane >= m_plane/2)
      return true;
    else
      return false;
}  

bool 
SPController::turnUp(int src_index, int dst_index){
    if(src_index < dst_index && dst_index - src_index <= m_index/2)
      return true;
    else if (src_index > dst_index && src_index- dst_index >= m_index/2)
      return true;
    else
      return false;
}
 

dpid_t 
SPController::getDpidByIndex(int i)
{
    Ptr<OFSwitch13Device> ofdev = m_switches.Get(i)->GetObject<OFSwitch13Device>();
    return ofdev->GetDpId();
}
