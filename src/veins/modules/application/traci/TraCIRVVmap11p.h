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

#ifndef TraCIRVVmap11p_H
#define TraCIRVVmap11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * Small IVC Demo using 11p
 */
class TraCIRVVmap11p : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);

		class Statistics {
            public:
                //Basic stats
                cOutVector xCoord;
                cOutVector yCoord;
                cOutVector zCoord;

                void initialize();
        };
		enum WaveApplMessageKinds {
		            SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
		            SEND_BEACON_EVT,
		            SEND_HELLO
		        };

	protected:
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
		double probCH;
		//create the maps of neighbors
		std::map<int, double> neighborsdDistCalc;
		std::map<int, simtime_t> neighborsdTime;
		Statistics statistics;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		void sendRVVMessage(std::string type, int toNode);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleSelfMsg(cMessage* msg);
		virtual void handleLowerMsg(cMessage* msg);
		void updateInfo(WaveShortMessage* data);
		void changeNodeState();
		double calcUtility(double sqrD);
};

#endif
