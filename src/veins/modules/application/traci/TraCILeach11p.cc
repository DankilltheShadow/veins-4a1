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

#include "veins/modules/application/traci/TraCILeach11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCILeach11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(TraCILeach11p);

void TraCILeach11p::Statistics::initialize()
{
    numCH = 0;
    numON = 0;
    numFN = 0;
    meanCluster=0;
    meanCHutility=0;
}

void TraCILeach11p::Statistics::recordScalars(cSimpleModule& module, double div)
{
    if(numCH!=0){
        module.recordScalar("meanClusterNodes", meanCluster/numCH);
        module.recordScalar("meanClusterHeadUtility", meanCHutility/numCH);
    }else{
        module.recordScalar("meanClusterNodes", 0);
        module.recordScalar("meanClusterHeadUtility", 0);
    }
    module.recordScalar("NumberofCH", numCH/div);
    module.recordScalar("NumberofON", numON/div);
    module.recordScalar("NumberofFN", numFN/div);
}

void TraCILeach11p::collectStatistics(){
    if(fmod(simTime().dbl(),(timeHello/2))==0){
        numCollStats++;
        if(std::string(par("Car_State").stringValue())=="CH"){
            statistics.numCH++;
            statistics.meanCluster += ownON.size();
            double utility=0;
            for(auto it = ownON.begin();it != ownON.end(); it++){
                const int ON = *it;
                utility += nodesUtility[ON];
            }
            if(ownON.size()!=0)
                statistics.meanCHutility = utility/(10*ownON.size());
        }else if(std::string(par("Car_State").stringValue())=="ON"){
            statistics.numON++;
        }else if(std::string(par("Car_State").stringValue())=="FN"){
            statistics.numFN++;
        }
    }
}

double TraCILeach11p::calcUtility(double sqrD){
    double sigmaQ=FWMath::dBm2mW(-110); //converto i dBm in mW per sigma quadro
    double pDivSigma = 10/sigmaQ; // 10 mW
    double w = 20; //MHZ
    double g = 1/sqrD;

    return(w*log(1+(pDivSigma*g)));
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
        statistics.initialize();
        nextCHturn = false;
        double time;
        if(simTime().dbl() < 180){
            time=180;
        }else{
            time=simTime().dbl();
        }

        scheduleAt(offSetHello + time, sendHelloTimer);
	}
}

void TraCILeach11p::onBeacon(WaveShortMessage* wsm) {
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

    if(std::string(wsm->getName())=="Hello") {
        onHello(wsm);
    } else if(std::string(wsm->getName())=="AssReq" && wsm->getRecipientAddress()==myId) {
        onReq(wsm);
    } else if(std::string(wsm->getName())=="AssRef" && wsm->getRecipientAddress()==myId) {
        onRef(wsm);
    } else if(std::string(wsm->getName())=="deAss" && wsm->getRecipientAddress()==myId) {
        onDeA(wsm);
    } else if(std::string(wsm->getName())=="beacon" && wsm->getRecipientAddress()==myId) {
        onBeacon(wsm);
    } else if(std::string(wsm->getName())=="data" && wsm->getRecipientAddress()==myId) {
        onData(wsm);
    }
    delete(msg);
}

void TraCILeach11p::onDeA(WaveShortMessage* wsm) {
    if(std::string(par("Car_State").stringValue())=="CH"){
        ownON.remove(wsm->getSenderAddress());
    }
}

void TraCILeach11p::onRef(WaveShortMessage* wsm) {
    if(std::string(par("Car_State").stringValue())=="ON" && ownCH==wsm->getSenderAddress()){
        ownCH=-1;
        par("Car_State").setStringValue("FN");
    }
}

void TraCILeach11p::onReq(WaveShortMessage* wsm) {
    if(std::string(par("Car_State").stringValue())!="CH" || ownON.size()==capacity){
        //rifiuto
        sendRVVMessage("AssRef",wsm->getSenderAddress());
    }else{
        ownON.push_back(wsm->getSenderAddress());
        const double sqrDistance = this->curPosition.sqrdist(wsm->getSenderPos());
        nodesUtility[wsm->getSenderAddress()] = calcUtility(sqrDistance);
    }
}

void TraCILeach11p::onHello(WaveShortMessage* wsm) {
    if(std::string(par("Car_State").stringValue())=="FN"){
        //divento suo ON e lo informo
        par("Car_State").setStringValue("ON");
        ownCH = wsm->getSenderAddress();
        //aggiungo il mittente al mio CH
        sendRVVMessage("AssReq",ownCH);
    }
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
    for(auto it = ownON.begin();it != ownON.end(); it++){
        const int ON = *it;
        sendRVVMessage("AssRef",ON);
    }
    ownON.clear();
    sendRVVMessage("deAss",ownCH);
    ownCH=-1;
    if(fmod(nTurn,(1/probCH))==0){
        nextCHturn = false;
    }
    if((!nextCHturn) && (uniform(0,1) <= probCH/(1-probCH*(fmod(nTurn,(1/probCH)))))){
        par("Car_State").setStringValue("CH");
        nextCHturn = true;
        sendRVVMessage("Hello"); //send broadcast hello MSG.
    }else{
        par("Car_State").setStringValue("FN");
    }
    nTurn++;
}

void TraCILeach11p::finish() {
    statistics.recordScalars(*this, numCollStats);
}
