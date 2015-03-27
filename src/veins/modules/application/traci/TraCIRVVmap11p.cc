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

#include "veins/modules/application/traci/TraCIRVVmap11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCIRVVmap11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(TraCIRVVmap11p);

void TraCIRVVmap11p::Statistics::initialize() {
    xCoord.setName("Posizione x");
    yCoord.setName("Posizione y");
    zCoord.setName("Posizione z");
}

void TraCIRVVmap11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();

		//set hello messages
		sendHelloTimer = new cMessage("Hello!", SEND_HELLO);
		statistics.initialize();

        //simulate asynchronous channel access
        double offSetHello = dblrand() * (par("helloInterval").doubleValue()/2);
        offSetHello = offSetHello + floor(offSetHello/0.050)*0.050;

        scheduleAt(offSetHello + simTime(), sendHelloTimer);

        //graphic indentification. FN=red ON=blue CH=green
        findHost()->getDisplayString().updateWith("r=5,red");

        capacity = par("capacityCluster");
	}
}

double TraCIRVVmap11p::calcUtility(double sqrD){
    double sigmaQ=FWMath::dBm2mW(-110); //converto i dBm in mW per sigma quadro
    double pDivSigma = 10/sigmaQ; // 10 mW
    double w = 20; //MHZ
    double g = 1/sqrD;

    return(w*log(1+(pDivSigma*g)));
}

void TraCIRVVmap11p::changeNodeState(){
    double UCH = 358;
    double UON = 0;
    double denom = 0;
    for(auto const& p:neighborsdDistCalc){
        double X = calcUtility(p.second);
        denom ++;
        UCH += X;
        UON += X/10;
    }
    if(denom!=0){
        UON = UON/denom;
        UCH = UCH/((denom+1)*10);
    }
    if(UCH>UON){
        par("Car_State").setStringValue("CH");
    } else {
        par("Car_State").setStringValue("ON");
    }
}

void TraCIRVVmap11p::onBeacon(WaveShortMessage* wsm) {
}

void TraCIRVVmap11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIRVVmap11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIRVVmap11p::sendRVVMessage(std::string type, int toNode=-1) {
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM(type, dataLengthBits, channel, dataPriority, toNode,2);
    wsm->setSenderState(par("Car_State").stringValue());

    std::multimap<double, int> PrList;
    for (auto const& p : neighborsdDistCalc)
    {
        PrList.insert(std::pair<double,int>(p.second, p.first));
    }
    wsm->setPrefListArraySize(PrList.size());
    int i = 0;
    for (std::multimap<double,int>::iterator it=PrList.begin(); it!=PrList.end(); ++it)
    {
        wsm->setPrefList(i,(*it).second);
        i++;
    }
    wsm->setCapacity(capacity);
    sendWSM(wsm);
}

void TraCIRVVmap11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}
void TraCIRVVmap11p::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}
void TraCIRVVmap11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	if(fmod(simTime().dbl(),5)==0){
	    statistics.xCoord.record(this->curPosition.x);
        statistics.yCoord.record(this->curPosition.y);
        statistics.zCoord.record(this->curPosition.z);
	}

	for (auto it = neighborsdDistCalc.begin(); it != neighborsdDistCalc.end(); ){
        if (simTime() - neighborsdTime[it->first] > 2*par("helloInterval").doubleValue()){
            it = neighborsdDistCalc.erase(it);
        }else{
            it++;
        }
    }
}
void TraCIRVVmap11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

void TraCIRVVmap11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_HELLO: {
            changeNodeState();
            sendRVVMessage("Hello"); //send broadcast hello MSG.
            scheduleAt(simTime() + par("helloInterval").doubleValue(), sendHelloTimer);
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

void TraCIRVVmap11p::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (std::string(wsm->getName()) == "Hello") {
        updateInfo(wsm);
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

void TraCIRVVmap11p::updateInfo(WaveShortMessage* wsm) {
    int id = wsm->getSenderAddress();
    neighborsdDistCalc[id]=this->curPosition.sqrdist(wsm->getSenderPos());
    neighborsdTime[id]=wsm->getTimestamp();
}
