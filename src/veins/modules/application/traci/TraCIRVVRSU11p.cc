//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIRVVRSU11p.h"
#include <iostream>
#include <fstream>

using Veins::AnnotationManagerAccess;

Define_Module(TraCIRVVRSU11p);

void TraCIRVVRSU11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobi = dynamic_cast<BaseMobility*> (getParentModule()->getSubmodule("mobility"));
		ASSERT(mobi);
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);
		sentMessage = false;
		startMatching = new cMessage("Start!", SEND_MATCH);
		scheduleAt(simTime() + par("startMatching").doubleValue(), startMatching);

	}
}

void TraCIRVVRSU11p::onBeacon(WaveShortMessage* wsm) {

}

void TraCIRVVRSU11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobi->getCurrentPosition(), "blue"));

	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIRVVRSU11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIRVVRSU11p::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}

void TraCIRVVRSU11p::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (std::string(wsm->getName()) == "Plist") {
        onPreferenceList(wsm);
    } else if (std::string(wsm->getName()) == "beacon") {
        onBeacon(wsm);
    }
    else if (std::string(wsm->getName()) == "data") {
        onData(wsm);
    }
    else {
        DBG << "unknown message (" << wsm->getName() << ")  received\n";
    }
    delete(msg);
}

void TraCIRVVRSU11p::onPreferenceList(WaveShortMessage* wsm) {
    int id = wsm->getSenderAddress();
    std::vector<int> list;

    for (size_t i=0; i<wsm->getPrefListArraySize(); ++i){
        list.push_back(wsm->getPrefList(i));
    }
    if(std::string(wsm->getSenderState())=="CH"){
        PrefCHLists.insert(std::pair<int, std::vector<int>> (id, list));
        CHcapacity.insert(std::pair<int, int> (id, wsm->getCapacity()));
    }else{
        PrefONLists.insert(std::pair<int, std::vector<int>> (id, list));
    }
}

/*
 Si fermano i veicoli a circa 243. ultimo Hello a circa 263. A 265 circa il primo Plist. <<<280 fine Plist. 82 Nodi.*/
void TraCIRVVRSU11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_MATCH: {
            launchMatching();
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

typedef std::vector<int> PrefList;
typedef std::map<int, PrefList> PrefMap;
typedef std::multimap<int, int> Matching;

void TraCIRVVRSU11p::launchMatching() {

    PrefList ONunmatched;
    Matching Matched;
    for(auto const& iter : PrefONLists){
        ONunmatched.push_back(iter.first);
    }
    while(!ONunmatched.empty()){
        for(size_t it = 0; it<ONunmatched.size(); it++){
            int ON = ONunmatched[it];
            const PrefList &preflist = PrefONLists[ON];
            if(!preflist.empty()){
                const int &pCH = *preflist.begin();
                if(PrefCHLists.find(pCH)!=PrefCHLists.end()){
                    Matched.insert(std::pair<int,int>(pCH, ON));
                }else{
                    it--;
                }
                PrefONLists[ON].erase(PrefONLists[ON].begin());
            }else{
                ONunmatched.erase(ONunmatched.begin()+it);
            }
        }
        for(auto const& p : PrefCHLists){
            const int CH = p.first;
            std::pair <Matching::iterator, Matching::iterator> ret;
            ret = Matched.equal_range(CH);
            int CountON = Matched.count(CH);
            if( CountON > CHcapacity[CH] ){
                PrefList preflistCH = p.second;
                std::map<int, Matching::iterator> Order;
                int size = preflistCH.size();
                for( Matching::iterator it = ret.first; it != ret.second; it++ ){
                    bool found=false;
                    for( size_t itpCH = 0; itpCH < preflistCH.size(); itpCH++ ){
                        if( it->second == preflistCH[itpCH] ){
                            Order[itpCH] = it;
                            found = true;
                            break;
                        }
                    }
                    if(!found){
                        Order[size] = it;
                        size++;
                    }
                }
                for(int itC=0; itC < CountON-CHcapacity[CH]; itC++){
                    std::map<int, Matching::iterator>::reverse_iterator last= Order.rbegin();
                    Matched.erase(last->second);
                    bool found=false;
                    for(size_t itpB=0; itpB<ONunmatched.size(); itpB++){
                        if(ONunmatched[itpB]==last->second->second){
                            found=true;
                            break;
                        }
                    }
                    if(!found){
                        ONunmatched.push_back(last->second->second);
                    }
                    Order.erase(last->first);
                }
            }
            for(Matching::iterator it=ret.first; it!=ret.second; ++it){
                for(size_t itUn = 0; itUn<ONunmatched.size(); itUn++){
                    if(ONunmatched[itUn]==it->second){
                        ONunmatched.erase(ONunmatched.begin()+itUn);
                    }
                }
            }
        }

    }
    std::ofstream myfile;
    myfile.open ("matching.txt");
    Matching::iterator iter;
    for (iter=Matched.begin(); iter!=Matched.end(); ++iter)
    {
        myfile<<(*iter).first<<"->"<<(*iter).second<<"\n";
    }
    myfile.close();

}

