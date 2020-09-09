#include "dumphelper.h"

#include <iostream>
using namespace std;


DumpHelper::DumpHelper(uint16_t node_number)
{
    _node_number = node_number;
}

void DumpHelper::dumpDpidList(const NodeContainer switches)
{
    for(uint16_t i = 0; i < switches.GetN(); i++) {
        Ptr<OFSwitch13Device> ofdev = switches.Get(i)->GetObject<OFSwitch13Device>();
        cout<<"dumpDpidList: dpid of dev "<<i <<" is "<<ofdev->GetDpId()<<endl;
    }
}

void DumpHelper::dumpIndexInfo(const nodeInfo_t* indexInfo)
{
    for(uint16_t i = 0; i < _node_number; i++) {
        nodeInfo_t a = indexInfo[i];
        cout<<"dumpIndexInfo: dev: "<<i <<", plane = "<<a.plane<<", index= "<<a.index<<endl;
    }
} 

void DumpHelper::dumpDpidInfo(const nodeInfo_t* dpidInfo)
{
    for(uint16_t i = 1; i < _node_number + 1; i++) {
        nodeInfo_t a = dpidInfo[i];
        cout<<"dumpDpidInfo: dpid: "<<i <<", plane = "<<a.plane<<", index= "<<a.index<<endl;
    }

}


 
void DumpHelper::dumpIndexAdj(double** const indexAdj)
{
    for(uint16_t i = 0; i < _node_number; i++) {
        for(uint16_t j = 0; j < _node_number; j++){
            if(indexAdj[i][j] != -1){
                cout<<"dumpIndexAdj: dev: "<<i <<" to dev: "<<j<<" cost= "<<indexAdj[i][j]<<endl;
            }
        }
    }
} 
 
void DumpHelper::dumpDpidAdj(double** const dpidAdj)
{
   
    for(uint16_t i = 1; i < _node_number+1; i++) {
        bool find = false;
        for(uint16_t j = 1; j < _node_number+1; j++){
            if(dpidAdj[i][j] != -1){
                find =true;
                cout<<"dumpIndexAdj: dpid: "<<i <<" to dpid: "<<j<<" cost= "<<dpidAdj[i][j]<<endl;
            }
        }
        if(!find)  cout<<"dumpIndexAdj: dpid: "<<i <<" has no neighbor!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    }
  

} 

void DumpHelper::dumpDevPortMap(int** const devPortMap)
{
    for(uint16_t i = 0; i < _node_number; i++) {
        for(uint16_t j = 0; j < _node_number; j++){
            if(devPortMap[i][j] != -1){
                cout<<"dumpDevPortMap: dev: "<<i <<" to dev: "<<j<<" port= "<<devPortMap[i][j]<<endl;
            }
        }
    }
}

void DumpHelper::dumpDpidPortMap(int** const dpidPortMap)
{
    for(uint16_t i = 1; i < _node_number+1; i++) {
        for(uint16_t j = 1; j < _node_number+1; j++){
            if(dpidPortMap[i][j] != -1){
                cout<<"dumpDpidPortMap: dpid: "<<i <<" to dpid: "<<j<<" port= "<<dpidPortMap[i][j]<<endl;
            }
        }
    }
}