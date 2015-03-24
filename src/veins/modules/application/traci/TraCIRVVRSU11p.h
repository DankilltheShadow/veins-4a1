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

typedef std::vector<int> PrefList;
typedef std::map<int, PrefList> PrefMap;
typedef std::multimap<int, int> Matching;

/**
 * Small RSU Demo using 11p
 */
class TraCIRVVRSU11p : public BaseWaveApplLayer {
	public:
        class Statistics {
            public:
                //Basic stats
                cOutVector numCH;
                cOutVector numON;
                cOutVector numFN;
                cOutVector numCHLost;
                cOutVector meanClusterSize;
                cOutVector stddevCluster;
                cOutVector varianceCluster;
                //Neighbors stats
                cOutVector meanNeighbors;
                cOutVector stddevNeighbors;
                cOutVector varianceNeighbors;
                //CH stats
                cOutVector totalCHUtility;
                cOutVector meanCHUtility;
                cOutVector stddevCHUtility;
                cOutVector varianceCHUtility;
                cOutVector totalExpCHUtility;
                cOutVector meanExpCHUtility;
                cOutVector stddevExpCHUtility;
                cOutVector varianceExpCHUtility;
                //ON stats
                cOutVector totalONUtility;
                cOutVector meanONUtility;
                cOutVector stddevONUtility;
                cOutVector varianceONUtility;
                cOutVector totalExpONUtility;
                cOutVector meanExpONUtility;
                cOutVector stddevExpONUtility;
                cOutVector varianceExpONUtility;

                void initialize();
        };

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
		PrefMap PrefCHLists;
		PrefMap PrefONLists;
		Matching Matched;
		std::map<int, Coord> nodesCoord;
		std::map<int, int> CHcapacity;
		std::map<int, simtime_t> nodesTime;
		double intervalDeletion;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleLowerMsg(cMessage* msg);
		virtual void handleSelfMsg(cMessage* msg);
        void updatePreferenceList(WaveShortMessage* data);
        double calcUtility(double sqrD);
        void orgStatistic();
        //void adjustPrefList();
        void deleteOldNodes();
        Matching RVV(Matching Mz);
        Matching foundBP(Matching m, PrefMap fPrefCHLists, PrefMap fPrefONLists);
        bool add(int a, int r, std::map<int, int> &S, Matching &m);
        Matching blockingAgent(int agent, std::map<int, int> S, Matching m);
        void satisfy(int CH, int ON, std::map<int, int> &S, Matching &m );
};

#endif
