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

#ifndef TraCIRVVRSU11p_H
#define TraCIRVVRSU11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

using Veins::AnnotationManager;

/**
 * Small RSU Demo using 11p
 */
class TraCIRVVRSU11p : public BaseWaveApplLayer {
	public:
        class Statistics {
            public:
                double numCH;
                double numON;
                double numFN;
                double meanCluster;
                double sigmaCluster;

                void initialize();
                void watch(cSimpleModule& module);
                void recordScalars(cSimpleModule& module);
        };

        virtual void finish();
		virtual void initialize(int stage);

		enum WaveApplMessageKinds {
            SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
            SEND_BEACON_EVT,
            SEND_MATCH
        };
	protected:
		Statistics statistics;
		AnnotationManager* annotations;
		BaseMobility* mobi;
		bool sentMessage;
		cMessage* startMatching;
		//create the maps of Preference lists
		std::map<int, std::vector<int>> PrefCHLists;
		std::map<int, std::vector<int>> PrefONLists;
		std::map<int, int> CHcapacity;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleLowerMsg(cMessage* msg);
		virtual void handleSelfMsg(cMessage* msg);
        void onPreferenceList(WaveShortMessage* data);
        void launchMatching();
};

#endif
