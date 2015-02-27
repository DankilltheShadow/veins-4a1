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
		WATCH_MAP(PrefCHLists);
		WATCH_MAP(PrefONLists);
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
    int list[wsm->getPrefListArraySize()];
    for (size_t i=0; i<wsm->getPrefListArraySize(); ++i){
        list[i]=wsm->getPrefList(i);
    }
    if(std::string(wsm->getSenderState())=="CH"){
        PrefCHLists[id]=list;
    }else{
        PrefONLists[id]=list;
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

void TraCIRVVRSU11p::launchMatching() {
    /*std::map<int, int> ONlist;
    std::map<int, int> CHlist;
    bool Matching[PrefCHLists.size()][PrefONLists.size()]={false};
    int i=0;
    for(auto const& iter : PrefONLists) {
        ONlist[i] = iter.first;
        i++;
    }
    i=0;
    for(auto const& iter : PrefCHLists) {
            CHlist[i] = iter.first;
            i++;
        }
    size_t ONlistSize = sizeof(ONlist) / sizeof(int);
    while(ONlistSize!=0 or PrefONLists.size()!=0){
        for(size_t i=0;i<ONlistSize;i++){
            if(sizeof(PrefONLists[ONlist[i]])/sizeof(int)!=0){
                for(size_t j=0;j<sizeof(CHlist)/sizeof(int);j++){
                    if(PrefONLists[ONlist[i]][0]==CHlist[j]){
                        Matching[j][i]=true;
                    }
                }
            }
        }
    }*/
    std::map <int,int> ONMatching;
    for(auto const& iter : PrefONLists) {
        ONMatching[iter.first]=iter.second[0];

    }

}

