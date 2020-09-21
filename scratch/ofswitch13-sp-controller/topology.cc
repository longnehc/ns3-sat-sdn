
#include "topology.h"

 
TopoHelper::TopoHelper(uint16_t nSat, double latborder, nodeInfo_t* indexInfo, nodeInfo_t* dpidInfo,  NetDeviceContainer* switchPorts,
   double** indexAdj, double** dpidAdj, int** dpidPortMap, int** devPortMap)
{ 
    _nSat = nSat;
    _latborder = latborder;
    _indexInfo = indexInfo;
    _dpidInfo = dpidInfo;
    _switchPorts = switchPorts;
    _dpidPortMap = dpidPortMap;
    _devPortMap = devPortMap;
    _indexAdj = indexAdj;
    _dpidAdj = dpidAdj;
    IpBias = 0;
}

vector<PolarSatPosition> 
TopoHelper::SatNodeInit(int _nPlane, int _nIndex, uint16_t _altitude, double _incl){
  vector<PolarSatPosition> satPositions;
  SatGeometry sg();
  for (int i=0; i<_nPlane; i++){
    for(int j=0; j<_nIndex; j++){
      double lon = 22.5 * i;    // double lon = 31.6*i; 31.6 for iridium, 6 planes - 2pi, so 180 / 8 = 22.5
      double alpha =  fmod(5 * i + 40 * j, 360); //360/(8*9)=5 360 / 9 = 40
      PolarSatPosition psp = PolarSatPosition(_altitude, _incl, lon, alpha, i, j);
      //cout<<"Inserting node "<<i<<","<<j<<" lon: "<<lon<<" alpha: "<< alpha<<endl;
      satPositions.push_back(psp);
      //cout<<"The lat is: "<<RAD_TO_DEG(sg.get_latitude(psp.coord(0)))<<", the long is "<<RAD_TO_DEG(sg.get_longitude(psp.coord(0),0))<<endl;
      nodeInfo_t nit;
      nit.plane = i;
      nit.index = j;
      _indexInfo[i * _nIndex + j] = nit;
    }
  }
  return satPositions;
}

string
TopoHelper::IpAddressConstructor(){
  ostringstream oss;
  oss << "192.1."<<IpBias<<".0"; 
  //cout<<oss.str().c_str()<<endl;
  IpBias++;
  return oss.str();
}


void 
TopoHelper::SatLinkInit(vector<PolarSatPosition> satPositions, NodeContainer switches, int _nPlane, int _nIndex){
  SatGeometry sg(_latborder);
  for(int i = 0; i < _nPlane; i++){
    for(int j = 0; j < _nIndex; j++){
      int node_index = i * _nIndex + j;
       //inter-plane isl
      int up_j = (j != _nIndex - 1) ? j + 1 : 0;
      int up_node_index = i * _nIndex + up_j;
  
        double delay1 = sg.propdelay(satPositions[node_index].coord(0), satPositions[up_node_index].coord(0));
        buildLink(node_index, up_node_index, delay1 * 1000, switches);
        _indexAdj[node_index][up_node_index] = delay1 * 1000;
        _indexAdj[up_node_index][node_index] = delay1 * 1000;
       // cout<<"Delays between" <<node_index<<" "<<up_node_index<<"("<<i<<","<<j<<") -> ("<<i<<","<<up_j<<" ) is "<<delay1<<endl;
      //} 
      //intra-plane isl
      int right_i = (i != _nPlane - 1) ? i + 1 : 0;
      int right_node_index = right_i * _nIndex + j;
      if(!sg.inPolar(satPositions[node_index].coord(0)) && !sg.inPolar(satPositions[right_node_index].coord(0))) {
          double delay4 = sg.propdelay(satPositions[node_index].coord(0), satPositions[right_node_index].coord(0));
          buildLink(node_index, right_node_index, delay4 * 1000, switches);
          _indexAdj[node_index][right_node_index] = delay4 * 1000;
          _indexAdj[right_node_index][node_index] = delay4 * 1000;
       //   cout<<"Delays between" <<node_index<<" "<<right_node_index<<"("<<i<<","<<j<<") -> ("<<right_i<<","<<j<<" ) is "<<delay4<<endl;
      }
     
    }
  }
      
        Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
        
        uint16_t nport = 50000;
        ApplicationContainer sinkApp;
        Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), nport));
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);
        sinkApp.Add(sinkHelper.Install(switches.Get(9)));
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (10.0));

        OnOffHelper clientHelper ("ns3::TcpSocketFactory", Address ());
        clientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        clientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer clientApps;
        AddressValue remoteAddress(InetSocketAddress (switchAddrMap[9], nport));
        clientHelper.SetAttribute("Remote",remoteAddress);
        clientApps.Add(clientHelper.Install(switches.Get(69)));

        clientApps.Start(Seconds(1.0));
        clientApps.Stop (Seconds (10.0));
   
 
}

void 
TopoHelper::buildLink (int src, int dst, double delay, NodeContainer switches)
{
      CsmaHelper csmaH;
      csmaH.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("20Mbps")));
      csmaH.SetChannelAttribute ("Delay", StringValue (double2string(delay)+"ms"));
      //add link n0---n1
      NodeContainer nc = NodeContainer(switches.Get(src),switches.Get(dst));  
      NetDeviceContainer p2pD = csmaH.Install(nc); 
      _switchPorts[src].Add(p2pD.Get(0));
      _switchPorts[dst].Add(p2pD.Get(1));

      Ipv4InterfaceContainer isrcidst;
      if(switchAddrMap.find(src) == switchAddrMap.end() || 
        switchAddrMap.find(dst) == switchAddrMap.end()){
        ipv4.SetBase ((Ipv4Address)(IpAddressConstructor().c_str()), "255.255.255.0");
        isrcidst = ipv4.Assign (p2pD);
        if(switchAddrMap.find(src) == switchAddrMap.end())
          switchAddrMap[src] = isrcidst.GetAddress(0);
        if(switchAddrMap.find(dst) == switchAddrMap.end() )
          switchAddrMap[dst] = isrcidst.GetAddress(1);
        cout<<"src addr: "<<src<<","<<isrcidst.GetAddress(0)<<endl;
        cout<<"dst addr: "<<dst<<","<<isrcidst.GetAddress(1)<<endl;
      }
 
      updatePortMap(src, dst);
}


void 
TopoHelper::updatePortMap(int dev1, int dev2)
{
    
    uint32_t portNum = _switchPorts[dev1].GetN();
    _devPortMap[dev1][dev2] = portNum;
//    std::cout<<"updatePortMap: dev_id= "<< dev1<<" to dev_id "<<dev2<<"'s port is "<<portNum<<std::endl;

    uint32_t portNum2 = _switchPorts[dev2].GetN();
    _devPortMap[dev2][dev1] = portNum2;
//    std::cout<<"updatePortMap: dev_id= "<< dev2<<" to dev_id "<<dev1<<"'s port is "<<portNum2<<std::endl;
}



dpid_t 
TopoHelper::getDpidByIndex(int i, NodeContainer switches)
{
    Ptr<OFSwitch13Device> ofdev = switches.Get(i)->GetObject<OFSwitch13Device>();
    return ofdev->GetDpId();
}

void 
TopoHelper::dpidInfoConstruct(NodeContainer switches)
{
    for(int i = 0; i < _nSat; i++){
        dpid_t _dpid = getDpidByIndex(i, switches);
        _dpidInfo[_dpid] = _indexInfo[i];
    }
}

void 
TopoHelper::dpidAdjConstruct(NodeContainer switches)
{  
    for(int i = 0; i < _nSat; i++){
        for(int j = 0; j < _nSat; j++){
            dpid_t dpid_i = getDpidByIndex(i, switches);
            dpid_t dpid_j = getDpidByIndex(j, switches);
            _dpidAdj[dpid_i][dpid_j] = _indexAdj[i][j];
        }
    }
}

void 
TopoHelper::dpidPortMapConstruct(NodeContainer switches)
{  
    for(int i = 0; i < _nSat; i++){
        for(int j = 0; j < _nSat; j++){
            dpid_t dpid_i = getDpidByIndex(i, switches);
            dpid_t dpid_j = getDpidByIndex(j, switches);
            _dpidPortMap[dpid_i][dpid_j] = _devPortMap[i][j];
        }
    }
} 


void 
TopoHelper::updatetopo(Ptr<SPController> ctrl, Ptr<SPController> spctrl, vector<PolarSatPosition> satPositions,
  NodeContainer switches, int _nPlane, int _nIndex){
  int now = Simulator::Now().GetSeconds();
  cout<<"Update topology invoked at "<<now<<endl;
  SatGeometry sg(_latborder);
  for(int i = 0; i < _nSat; i++)
        for(int j = 0; j < _nSat; j++)
            _indexAdj[i][j] = -1;
  for(int i = 0; i < _nSat + 1; i++)
        for(int j = 0; j < _nSat + 1; j++)
            _dpidAdj[i][j] = -1;

  for(int i = 0; i < _nPlane; i++){
    for(int j = 0; j < _nIndex; j++){
      int node_index = i * _nIndex + j;
       //inter-plane isl
      int up_j = (j != _nIndex - 1) ? j + 1 : 0;
      int up_node_index = i * _nIndex + up_j; 
      double delay1 = sg.propdelay(satPositions[node_index].coord(now), satPositions[up_node_index].coord(now));
      _indexAdj[node_index][up_node_index] = delay1 * 1000;
      _indexAdj[up_node_index][node_index] = delay1 * 1000;
       // cout<<"Delays between" <<node_index<<" "<<up_node_index<<"("<<i<<","<<j<<") -> ("<<i<<","<<up_j<<" ) is "<<delay1<<endl;
      //} 
      //intra-plane isl
      int right_i = (i != _nPlane - 1) ? i + 1 : 0;
      int right_node_index = right_i * _nIndex + j;
      if(!sg.inPolar(satPositions[node_index].coord(now)) && !sg.inPolar(satPositions[right_node_index].coord(now))) {
          double delay4 = sg.propdelay(satPositions[node_index].coord(now), satPositions[right_node_index].coord(now));
          _indexAdj[node_index][right_node_index] = delay4 * 1000;
          _indexAdj[right_node_index][node_index] = delay4 * 1000;
       //   cout<<"Delays between" <<node_index<<" "<<right_node_index<<"("<<i<<","<<j<<") -> ("<<right_i<<","<<j<<" ) is "<<delay4<<endl;
      }
    }
  }
  dpidAdjConstruct(switches);
  ctrl->ImportDpidAdj(_dpidAdj, _nSat + 1);
  spctrl->ImportDpidAdj(_dpidAdj, _nSat + 1);
  Simulator::Schedule (MilliSeconds (1000), &TopoHelper::updatetopo, this, ctrl, spctrl, satPositions, switches, _nPlane, _nIndex);
}

/*
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
*/