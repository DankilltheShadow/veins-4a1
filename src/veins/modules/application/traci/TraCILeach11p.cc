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

#include "veins/modules/application/traci/TraCIRVV11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCILeach11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(TraCILeach11p);

void TraCILeach11p::Statistics::initialize()
{
    numAssVector.setName("ON Associati");
    numCH.setName("CH state");
    numON.setName("ON state");
    numFN.setName("FN state");
    xCoord.setName("Posizione x");
    yCoord.setName("Posizione y");
}

void TraCILeach11p::collectStatistics(){
    if(fmod(simTime().dbl(),(timeHello/2))==0){
        numCollStats++;
    }
}

void TraCILeach11p::initialize(int stage) {
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
		timeHello = par("helloInterval").doubleValue();
        //simulate asynchronous channel access
        double offSetHello = dblrand() * (timeHello/2);
        offSetHello = offSetHello + floor(offSetHello/0.050)*0.050;

        capacity = par("capacityCluster");
        nTurn=0;
        //choose random role
        probCH = par("Prob_CH").doubleValue();
        numCollStats = 0;

        if(simTime().dbl() < 180){
            double time=180;
        }else{
            double time=simTime().dbl();
        }

        scheduleAt(offSetHello + time, sendHelloTimer);
	}
}

void TraCILeach11p::onBeacon(WaveShortMessage* wsm) {
}

void TraCILeach11p::onHello(WaveShortMessage* wsm) {
    if( std::string(par("Car_State").stringValue())=="FN"){
        //divento suo ON e lo informo
        par("Car_State").setStringValue("ON");
        //aggiungo il mittente al mio CH
        related.push_back(wsm->getSenderAddress());
        sendRVVMessage("AssReq");
    }
}

void TraCILeach11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCILeach11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCILeach11p::sendRVVMessage(std::string type, int toNode=-1) {
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM(type, dataLengthBits, channel, dataPriority, toNode,2);
    wsm->setSenderState(par("Car_State").stringValue());
    sendWSM(wsm);
}

void TraCILeach11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if(simTime() == simulation.getWarmupPeriod()){
	    traciVehicle->setSpeed(0.0);
	}
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}
void TraCILeach11p::handleParkingUpdate(cObject* obj) {
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
void TraCILeach11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	// stopped for for at least 10s?
	if (mobility->getSpeed() < 1) {
	    double appTime= 2.5 * timeHello;
		if (simTime() - lastDroveAt >= appTime) {
		    collectStatistics();
		}
	}
	else {
		lastDroveAt = simTime();
	}
}
void TraCILeach11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

void TraCILeach11p::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    switch (std::string(wsm->getName())) {
        case "Hello": {
            onHello(wsm);
            break;
        }
        case "AssReq": {
            onReq(wsm);
            break;
        }
        case "AssRes": {
            onRes(wsm);
            break;
        }
        case "beacon": {
            onBeacon(wsm);
            break;
        }
        case "data": {
            onData(wsm);
            break;
        }
        default: {
            DBG << "unknown message (" << wsm->getName() << ")  received\n";
            break;
        }
    }
    delete(msg);
}

void TraCILeach11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_HELLO: {
            newTurn();
            scheduleAt(simTime() + timeHello, sendHelloTimer);
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

void TraCILeach11p::newTurn(){
    ownON.clear();
    if(nTurn%(1/probCH)==0){
        nextCHTurn = false;
    }
    if((!nextCHTurn) && (uniform(0,1) <= probCH/(1-probCH*(nTurn%(1/probCH))))){
        par("Car_State").setStringValue("CH");
        nextCHTurn = true;
        sendRVVMessage("Hello"); //send broadcast hello MSG.
    }else{
        par("Car_State").setStringValue("FN");
    }
    nTurn++;
}
