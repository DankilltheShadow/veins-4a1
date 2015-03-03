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

using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCIRVVRSU11p::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

Define_Module(TraCIRVVRSU11p);

void TraCIRVVRSU11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobi = dynamic_cast<BaseMobility*> (getParentModule()->getSubmodule("mobility"));
		ASSERT(mobi);
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);
		sentMessage = false;
		//WATCH_MAP(PrefCHLists);
		//WATCH_MAP(PrefONLists);
		WATCH_MAP(dimPrefLists);
		WATCH_MAP(statePrefLists);
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
    //int list[wsm->getPrefListArraySize()];
    std::vector<int> list;
    for (size_t i=0; i<wsm->getPrefListArraySize(); ++i){
        //list[i]=wsm->getPrefList(i);
        list.push_back(wsm->getPrefList(i));
    }
    if(std::string(wsm->getSenderState())=="CH"){
        //PrefCHLists[id]=list;
        PrefCHLists.insert(std::pair<int, std::vector<int>> (id, list));
    }else{
        //PrefONLists[id]=list;
        PrefONLists.insert(std::pair<int, std::vector<int>> (id, list));
    }
    dimPrefLists[id]=wsm->getPrefListArraySize();
    statePrefLists[id]=wsm->getSenderState();
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
        for(PrefList::const_iterator it = ONunmatched.begin(); it!= ONunmatched.end(); ++it){
            const int &ON = *it;
            const PrefList &preflist = PrefONLists[ON];
            Matched.insert(std::pair<int,int>(*preflist.begin(), ON));
            PrefONLists[ON].erase(PrefONLists[ON].begin());
            //cancellare il primo elemento
        }
        for(auto const& p : PrefCHLists){
            const int CH = p.first;
            std::pair <Matching::iterator, Matching::iterator> ret;
            ret = Matched.equal_range(CH);
            int CountON = Matched.count(CH);
            if(CountON < 4){ //4 è scelto per ora da me per lo 0.25 del random CH
                for(Matching::iterator it=ret.first; it!=ret.second; ++it){
                    for(PrefList::const_iterator itUn = ONunmatched.begin(); itUn!= ONunmatched.end(); ++itUn){
                        if(*itUn==it->second){
                            ONunmatched.erase(itUn);
                        }
                    }
                }
            }else{
                PrefList preflistCH = p.second;
                while(CountON>3){
                    int leastON = *preflistCH.end();
                    for(Matching::iterator it=ret.first; it!=ret.second; ++it){
                        if(leastON == it->second){
                            Matched.erase(it->first);
                            bool found=false;
                            for(PrefList::const_iterator itUn = ONunmatched.begin(); itUn!= ONunmatched.end(); ++itUn){
                                if(*itUn==it->second){
                                    found = true;
                                }
                            }
                            if(!found){
                                ONunmatched.push_back(leastON);
                            }
                            CountON--;
                        }
                    }
                    preflistCH.pop_back();
                }
            }

        }

    }

}

