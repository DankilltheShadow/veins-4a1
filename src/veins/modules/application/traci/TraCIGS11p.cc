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

#include "veins/modules/application/traci/TraCIGS11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCIGS11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(TraCIGS11p);

void TraCIGS11p::initialize(int stage) {
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

        //simulate asynchronous channel access
        double offSetHello = dblrand() * (par("helloInterval").doubleValue()/2);
        offSetHello = offSetHello + floor(offSetHello/0.050)*0.050;

        scheduleAt(offSetHello + simTime(), sendHelloTimer);

        //graphic indentification. FN=red ON=blue CH=green
        findHost()->getDisplayString().updateWith("r=5,red");

        capacity = par("capacityCluster");

        //choose random role
        double probCH = par("Prob_CH").doubleValue();
        if(uniform(0,1)<=probCH){
            par("Car_State").setStringValue("CH");
            findHost()->getDisplayString().updateWith("r=6,green");
        }
	}
}

void TraCIGS11p::onBeacon(WaveShortMessage* wsm) {
}

void TraCIGS11p::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIGS11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIGS11p::sendRVVMessage(std::string type, int toNode=-1) {
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM(type, dataLengthBits, channel, dataPriority, toNode,2);
    wsm->setSenderState(par("Car_State").stringValue());
    sendWSM(wsm);
}

void TraCIGS11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
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
void TraCIGS11p::handleParkingUpdate(cObject* obj) {
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
void TraCIGS11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	// stopped for for at least 10s?
	if (mobility->getSpeed() < 1) {
	    double appTime= 2.5 * par("helloInterval").doubleValue();
		if (simTime() - lastDroveAt == appTime) {
		    std::multimap<double, int> PrList;
		    for (auto const& p : neighborsdTime)
            {
                if(lastDroveAt-p.second >= appTime){
                    neighborsdCoord.erase(p.first);
                }else{
                    double dDistance = this->curPosition.distance(neighborsdCoord[p.first]);
                    PrList.insert(std::pair<double,int>(dDistance, p.first));
                }
            }

		    t_channel channel = dataOnSch ? type_SCH : type_CCH;
            WaveShortMessage* wsm = prepareWSM("Plist", dataLengthBits, channel, dataPriority, -9,2);
            wsm->setPrefListArraySize(PrList.size());
            int i = 0;
            std::multimap<double,int>::iterator it;
            for (it=PrList.begin(); it!=PrList.end(); ++it)
            {
                wsm->setPrefList(i,(*it).second);
                i++;
            }
            wsm->setSenderState(par("Car_State").stringValue());
            wsm->setCapacity(capacity);
            sendWSM(wsm);
		}
	}
	else {
		lastDroveAt = simTime();
	}
}
void TraCIGS11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

void TraCIGS11p::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_HELLO: {
            sendRVVMessage("Hello"); //send broadcast hello MSG.
            if(simTime() - lastDroveAt <= par("helloInterval").doubleValue()){
                scheduleAt(simTime() + par("helloInterval").doubleValue(), sendHelloTimer);
            }
            break;
        }
        default: {
            if (msg)
                BaseWaveApplLayer::handleSelfMsg(msg);
            break;
        }
    }
}

void TraCIGS11p::handleLowerMsg(cMessage* msg) {

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

void TraCIGS11p::updateInfo(WaveShortMessage* wsm) {
    if(std::string(par("Car_State").stringValue()) != wsm->getSenderState()){
        int id = wsm->getSenderAddress();
        neighborsdCoord[id]=wsm->getSenderPos();
        neighborsdTime[id]=wsm->getTimestamp();
    }

}
