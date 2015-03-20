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

#ifndef TraCILeach11p_H
#define TraCILeach11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * Small IVC Demo using 11p
 */
class TraCILeach11p : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);

		enum WaveApplMessageKinds {
		            SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
		            SEND_BEACON_EVT,
		            SEND_HELLO
		        };

        class Statistics {
           public:
               double numCH;
               double numON;
               double numFN;
               double meanCluster;

               void initialize();
               void recordScalars(cSimpleModule& module, double div);
        };

	protected:
        Statistics statistics;
		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		AnnotationManager* annotations;
		simtime_t lastDroveAt;
		bool sentMessage;
		bool isParking;
		bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;
		static const simsignalwrap_t neighbors;
		cMessage* sendHelloTimer;
		int capacity;
		double nTurn;
		double probCH;
		double timeHello;
		bool nextCHturn;
		int numCollStats;
		int ownCH;
		std::list<int> ownON;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		virtual void onHello(WaveShortMessage* wsm);
		virtual void onReq(WaveShortMessage* wsm);
		virtual void onRef(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		void sendRVVMessage(std::string type, int toNode);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleSelfMsg(cMessage* msg);
		virtual void handleLowerMsg(cMessage* msg);
		void newTurn();
		void finish();
		void collectStatistics();
};

#endif
